"""Serve one OpenCV camera as JPEG over ZeroMQ, in Unitree's image_server format.

Run this on the computer the camera is plugged into. Each frame is one ZeroMQ
PUB message holding a single JPEG, the same as Unitree's ``image_server.py``,
so hiRobot's ``unitree_image_server`` camera driver (or any image_server client)
reads it. For example, for a RealSense D435i on a Jetson::

    python3 camera_image_server.py \\
        --device /dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_435i_Intel_R__RealSense_TM__Depth_Camera_435i-video-index0 \\
        --width 640 --height 480 --fps 30

A D435i exposes several V4L2 nodes (depth, infrared, colour, metadata); the
colour stream is ``video-index0`` under ``/dev/v4l/by-id/``. The camera is only
read; nothing on the robot moves. Needs only ``opencv-python`` and ``pyzmq``
(Python 3.8 or newer, no hiRobot install).
"""

from __future__ import annotations

import argparse
import struct
import time


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--device", default="0",
                    help="V4L2 device path or index (default 0)")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=float, default=30.0, help="requested camera frame rate")
    ap.add_argument("--port", type=int, default=5555)
    ap.add_argument("--quality", type=int, default=85, help="JPEG quality, 1-100")
    ap.add_argument("--header", action="store_true",
                    help="prefix struct.pack('dI', time, frame_id), like image_server Unit_Test")
    args = ap.parse_args()

    import cv2
    import zmq

    device = int(args.device) if args.device.isdigit() else args.device
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise SystemExit(f"cannot open camera {args.device}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)

    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.setsockopt(zmq.SNDHWM, 2)
    sock.bind(f"tcp://*:{args.port}")
    params = [int(cv2.IMWRITE_JPEG_QUALITY), args.quality]
    print(f"serving {args.device} on tcp://*:{args.port}", flush=True)

    frame_id, t_report, n_report, failures = 0, time.time(), 0, 0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                failures += 1
                if failures >= 50:
                    raise SystemExit(f"camera {args.device}: 50 failed reads in a row")
                time.sleep(0.02)
                continue
            failures = 0
            ok, jpg = cv2.imencode(".jpg", frame, params)
            if not ok:
                continue
            payload = jpg.tobytes()
            if args.header:
                payload = struct.pack("dI", time.time(), frame_id) + payload
            sock.send(payload)
            frame_id += 1
            n_report += 1
            now = time.time()
            if now - t_report >= 10.0:
                print(f"{frame.shape[1]}x{frame.shape[0]} {n_report / (now - t_report):.1f} fps, "
                      f"{len(payload) // 1024} KB/frame", flush=True)
                t_report, n_report = now, 0
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        sock.close(0)
        ctx.term()


if __name__ == "__main__":
    main()
