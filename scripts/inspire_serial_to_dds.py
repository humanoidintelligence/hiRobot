"""Bridge two Inspire RH56DFX hands on one RS485 bus to DDS with inspire_sdkpy.

Run this on the computer the hands' USB-serial adapter is plugged into (on an
H1 this is usually PC2, 192.168.123.162). It does what the stock
``Headless_driver_485_double.py`` does, but also publishes ``angle_act``, the
0-1000 finger angles that hiRobot's ``inspire_sdkpy`` hand driver reads. The stock script only publishes ``pos_act``.

Topics (inspire_sdkpy IDL, module ``inspire``):

* ``rt/inspire_hand/state/{l,r}``: ``inspire_hand_state`` (published here)
* ``rt/inspire_hand/ctrl/{l,r}``: ``inspire_hand_ctrl`` (subscribed here)

Starting the bridge moves nothing: it only resets the hands' error flags
(register 1004) and then reads. Fingers move only when a ctrl message arrives.

Needs inspire_sdkpy 1.0.0 (https://github.com/NaCl-1374/inspire_hand_ws, commit
fc75490 plus a per-hand callback patch; the script checks for it),
pymodbus 3.6.9 and unitree_sdk2py 1.0.1. See docs/setup/unitree.md,
"Inspire hands (H1)".

Usage::

    python scripts/inspire_serial_to_dds.py --serial /dev/ttyUSB0 --left-id 2 --right-id 1
"""

from __future__ import annotations

import argparse
import time


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--serial", default="/dev/ttyUSB0")
    ap.add_argument("--baudrate", type=int, default=115200)
    ap.add_argument("--left-id", type=int, default=2, help="Inspire ID of the left hand")
    ap.add_argument("--right-id", type=int, default=1, help="Inspire ID of the right hand")
    ap.add_argument("--all-states", action="store_true",
                    help="also publish force, current, error, status, temperature (slower)")
    args = ap.parse_args()

    from inspire_sdkpy import inspire_sdk_double

    # Upstream fc75490 subscribes both ctrl topics to one callback that writes
    # every message to *both* hand IDs, so a left-hand command also moves the
    # right hand. The patched copy has one callback per hand.
    if not hasattr(inspire_sdk_double.ModbusDataHandlerDouble, "write_registers_callback_l"):
        raise SystemExit(
            "inspire_sdkpy's ModbusDataHandlerDouble has no per-hand ctrl callbacks: "
            "rt/inspire_hand/ctrl/l would also drive the right hand. Apply the per-hand "
            "callback patch (docs/setup/unitree.md, 'Inspire hands (H1)') first.")

    states = [
        ("pos_act", 1534, 6, "short"),
        ("angle_act", 1546, 6, "short"),
    ]
    if args.all_states:
        states += [
            ("force_act", 1582, 6, "short"),
            ("current", 1594, 6, "short"),
            ("err", 1606, 3, "byte"),
            ("status", 1612, 3, "byte"),
            ("temperature", 1618, 3, "byte"),
        ]
    # device_id is [left, right]: the "/l" topics use the first ID.
    handler = inspire_sdk_double.ModbusDataHandlerDouble(
        device_id=[args.left_id, args.right_id], use_serial=True,
        serial_port=args.serial, baudrate=args.baudrate, states_structure=states)
    time.sleep(0.5)
    n, t0 = 0, time.perf_counter()
    try:
        while True:
            handler.read()
            n += 1
            time.sleep(0.001)
            if n % 500 == 0:
                print(f"{n / (time.perf_counter() - t0):.1f} Hz, {n} reads", flush=True)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
