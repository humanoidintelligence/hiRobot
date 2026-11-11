"""The Unitree G1's built-in microphone and speaker.

Both sit behind the G1's audio board (192.168.123.161), not the Jetson's sound
card: PC2's ALSA input records only zeros.

**Microphone.** The audio board multicasts raw PCM over UDP to
``239.168.123.161:5555``, 16 kHz mono int16, as in Unitree's
``example/g1/audio/g1_audio_client_example.cpp`` (unitree_sdk2, C++ only).
Measured on a G1 from a desktop on the robot LAN: one 5120-byte packet
(160 ms) at a time, 32200 B/s, noise floor RMS ~120. Receiving needs no SDK
and sends nothing. One G1 sent no packets at all, not even to PC2; another
streamed as soon as the group was joined, so a silent microphone is a robot
problem, not a host one.

**Speaker.** The SDK's ``voice`` RPC service
(``unitree_sdk2py.g1.audio.AudioClient``). ``PlayStream`` takes 16 kHz mono
int16 and returns in 15-40 ms, long before the audio has played: the robot
buffers it, and blocks sent back to back play without a gap. ``PlayStop``
flushes that buffer: with 6 s queued, a stop 2 s in cut the sound at ~2 s.
So :meth:`UnitreeG1Speaker.play` keeps its own clock for the playback time and
:meth:`UnitreeG1Speaker.stop` calls ``PlayStop``.
"""

from __future__ import annotations

import fcntl
import logging
import socket
import struct
import threading
import time
from typing import Any, Optional

from hirobot.drivers._sdk.unitree_sdk2 import init_channel
from hirobot.loading.sdk import require_sdk
from hirobot.devices.audio import (
    SAMPLE_WIDTH,
    QueuedMicrophone,
    SpeakerDriver,
    register_audio,
    resample,
)

logger = logging.getLogger(__name__)

#: Where the G1 audio board multicasts microphone PCM.
MIC_GROUP = "239.168.123.161"
MIC_PORT = 5555
#: The audio board's address; used to find the local address on the robot LAN.
AUDIO_BOARD_IP = "192.168.123.161"

#: unitree_sdk2py.rpc.internal.RPC_ERR_CLIENT_SEND: no service answered the request.
_RPC_ERR_CLIENT_SEND = 3102
_SIOCGIFADDR = 0x8915


def _interface_ip(network_interface: str) -> str:
    """The IPv4 address of a network interface (Linux)."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        packed = struct.pack("256s", network_interface.encode()[:15])
        try:
            addr = fcntl.ioctl(s.fileno(), _SIOCGIFADDR, packed)[20:24]
        except OSError as exc:
            raise OSError(f"network interface {network_interface!r} has no IPv4 address") from exc
    return socket.inet_ntoa(addr)


def _route_ip(dest: str) -> str:
    """The local address the kernel would use to reach ``dest`` (sends nothing)."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.connect((dest, 9))
        return s.getsockname()[0]


@register_audio("unitree_voice_mic")
class UnitreeG1Microphone(QueuedMicrophone):
    """The G1 microphone, received from the audio board's UDP multicast.

    Args:
        name: Microphone name (its key in ``robot.microphones``).
        network_interface: NIC on the robot LAN (e.g. ``eth0``); the group is
            joined on its address.
        local_ip: Join on this address instead. Without either, the address
            routed towards the audio board is used.
        group: Multicast group.
        port: UDP port.
        queue_s: Seconds of audio kept for a slow reader.
    """

    sample_rate = 16000

    def __init__(
        self,
        name: str,
        network_interface: Optional[str] = None,
        local_ip: Optional[str] = None,
        group: str = MIC_GROUP,
        port: int = MIC_PORT,
        queue_s: float = 10.0,
    ) -> None:
        super().__init__(name, queue_s=queue_s)
        self._network_interface = network_interface
        self._local_ip = local_ip
        self._group = group
        self._port = port
        self._sock: Optional[socket.socket] = None

    def _join_ip(self) -> str:
        if self._local_ip:
            return self._local_ip
        if self._network_interface:
            return _interface_ip(self._network_interface)
        return _route_ip(AUDIO_BOARD_IP)

    def _open(self) -> None:
        local = self._join_ip()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        try:
            # Other listeners (a second process, Unitree's own tools) may share the port.
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("", self._port))
            mreq = socket.inet_aton(self._group) + socket.inet_aton(local)
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
            sock.settimeout(0.2)
        except BaseException:
            sock.close()
            raise
        logger.info("microphone %s: joined %s:%d on %s", self.name, self._group, self._port, local)
        self._sock = sock

    def _receive(self) -> Optional[bytes]:
        try:
            data, _ = self._sock.recvfrom(65535)
        except socket.timeout:
            return None
        return data[: len(data) // SAMPLE_WIDTH * SAMPLE_WIDTH]

    def _close(self) -> None:
        if self._sock is not None:
            self._sock.close()
            self._sock = None


def _code(ret: Any) -> int:
    """The status code of an AudioClient call (a bare int or ``(code, data)``)."""
    return ret[0] if isinstance(ret, tuple) else ret


@register_audio("unitree_voice_speaker")
class UnitreeG1Speaker(SpeakerDriver):
    """The G1 speaker, through the SDK's ``voice`` service.

    Audio is sent in ``chunk_s`` blocks, at most ``lead_s`` seconds ahead of
    the playback clock, so a stop has little queued audio to flush and a long
    clip is not pushed into the robot all at once.

    Args:
        name: Speaker name (its key in ``robot.speakers``).
        network_interface: NIC on the robot LAN. If the body driver already
            initialised DDS, the SDK keeps that setting, so give both the same NIC.
        domain_id: DDS domain (``0`` on hardware).
        volume: Volume 0-100 to set on ``open()``; the previous volume comes back
            on ``close()``. ``None`` leaves the volume alone. 60 is clearly
            audible next to the robot, 30 is quiet.
        app_name: Stream owner name sent with every request.
        chunk_s: Seconds of audio per ``PlayStream`` request (the SDK example uses 3).
        lead_s: How far ahead of playback audio is sent.
        timeout_s: RPC timeout per request.
    """

    sample_rate = 16000

    def __init__(
        self,
        name: str,
        network_interface: Optional[str] = None,
        domain_id: int = 0,
        volume: Optional[int] = None,
        app_name: str = "hirobot",
        chunk_s: float = 1.0,
        lead_s: float = 1.0,
        timeout_s: float = 2.0,
    ) -> None:
        super().__init__(name)
        if volume is not None and not 0 <= volume <= 100:
            raise ValueError(f"speaker volume must be 0-100, got {volume}")
        self._network_interface = network_interface
        self._domain_id = domain_id
        self._volume = volume
        self._app_name = app_name
        self._chunk_bytes = max(1, int(chunk_s * self.sample_rate)) * SAMPLE_WIDTH
        self._lead_s = lead_s
        self._timeout_s = timeout_s
        self._client = None
        self._saved_volume: Optional[int] = None
        self._stop = threading.Event()
        self._play_lock = threading.Lock()
        self._stream = 0

    def open(self) -> None:
        if self._client is not None:
            return
        require_sdk(__package__, "UnitreeG1Speaker")
        try:
            from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
        except ImportError as exc:  # pragma: no cover - import guard
            raise ImportError(
                "UnitreeG1Speaker needs unitree_sdk2py "
                "(https://github.com/unitreerobotics/unitree_sdk2_python)."
            ) from exc
        init_channel("UnitreeG1Speaker", self._domain_id, self._network_interface)
        client = AudioClient()
        client.SetTimeout(self._timeout_s)
        client.Init()
        self._client = client
        code, data = client.GetVolume()
        if code != 0:
            # Like a camera without frames: the body stays usable, play() raises.
            logger.warning("speaker %s: GetVolume failed with code %s%s", self.name, code,
                           self._hint(code))
            return
        if self._volume is not None:
            self._saved_volume = int(data["volume"])
            code = _code(client.SetVolume(int(self._volume)))
            if code != 0:
                logger.warning("speaker %s: SetVolume(%s) failed with code %s",
                               self.name, self._volume, code)
                self._saved_volume = None

    @staticmethod
    def _hint(code: int) -> str:
        if code == _RPC_ERR_CLIENT_SEND:
            return " (no voice service answered; is the robot on and on this LAN?)"
        return ""

    def play(self, pcm: bytes, sample_rate: Optional[int] = None) -> float:
        if self._client is None:
            raise RuntimeError(f"speaker {self.name!r} is not open")
        pcm = resample(pcm, sample_rate or self.sample_rate, self.sample_rate)
        pcm = pcm[: len(pcm) // SAMPLE_WIDTH * SAMPLE_WIDTH]
        if not pcm:
            return 0.0
        bps = SAMPLE_WIDTH * self.sample_rate
        total = len(pcm) / bps
        with self._play_lock:
            self._stop.clear()
            self._stream += 1
            stream_id = f"{int(time.time() * 1000)}-{self._stream}"
            start: Optional[float] = None
            offset = 0
            while True:
                elapsed = 0.0 if start is None else time.monotonic() - start
                if self._stop.is_set():
                    self._client.PlayStop(self._app_name)
                    return min(elapsed, total)
                sent_s = offset / bps
                if offset < len(pcm) and sent_s - elapsed <= self._lead_s:
                    block = pcm[offset : offset + self._chunk_bytes]
                    code = _code(self._client.PlayStream(self._app_name, stream_id, block))
                    if code != 0:
                        self._client.PlayStop(self._app_name)
                        raise RuntimeError(
                            f"speaker {self.name!r}: PlayStream failed with code {code}"
                            f"{self._hint(code)}"
                        )
                    if start is None:
                        start = time.monotonic()
                    offset += len(block)
                    continue
                if offset >= len(pcm):
                    wait = total - elapsed
                    if wait <= 0:
                        return total
                else:
                    wait = sent_s - self._lead_s - elapsed
                self._stop.wait(max(0.001, wait))

    def stop(self) -> None:
        self._stop.set()

    def set_volume(self, volume: int) -> None:
        """Set the robot's speaker volume (0-100) now."""
        if self._client is None:
            raise RuntimeError(f"speaker {self.name!r} is not open")
        code = _code(self._client.SetVolume(int(volume)))
        if code != 0:
            raise RuntimeError(f"speaker {self.name!r}: SetVolume failed with code {code}")

    def close(self) -> None:
        if self._client is None:
            return
        self.stop()
        with self._play_lock:  # wait for a play() in progress to see the stop
            try:
                if self._saved_volume is not None:
                    self._client.SetVolume(self._saved_volume)
            finally:
                self._saved_volume = None
                self._client = None
