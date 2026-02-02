#!/usr/bin/env python3
import argparse
import os
import random
import sys
import time

try:
    import can
except ImportError:
    print("python-can is required. Install with: pip3 install python-can", file=sys.stderr)
    sys.exit(1)


def parse_args():
    parser = argparse.ArgumentParser(description="Flood CAN bus to provoke RX overflow on MCU.")
    parser.add_argument("--iface", default="can0", help="SocketCAN interface (default: can0)")
    parser.add_argument("--bitrate", type=int, default=None, help="Set bitrate via ip link (e.g., 500000)")
    parser.add_argument("--duration", type=float, default=5.0, help="Seconds to transmit (default: 5)")
    parser.add_argument("--id", dest="can_id", type=lambda x: int(x, 0), default=0x123,
                        help="Base CAN ID (default: 0x123)")
    parser.add_argument("--random-id", action="store_true", help="Randomize 11-bit IDs")
    parser.add_argument("--extended", action="store_true", help="Use 29-bit extended IDs")
    parser.add_argument("--dlc", type=int, default=8, help="Data length (0-8, default: 8)")
    parser.add_argument("--payload", default=None, help="Hex bytes, e.g. 0102030405060708")
    parser.add_argument("--gap-us", type=int, default=0,
                        help="Microseconds between frames (default: 0 = max rate)")
    return parser.parse_args()


def maybe_set_bitrate(iface, bitrate):
    if bitrate is None:
        return
    os.system(f"sudo ip link set {iface} down")
    os.system(f"sudo ip link set {iface} type can bitrate {bitrate}")
    os.system(f"sudo ip link set {iface} up")


def parse_payload(dlc, payload_hex):
    if payload_hex is None:
        return bytes(random.getrandbits(8) for _ in range(dlc))
    payload_hex = payload_hex.strip()
    if len(payload_hex) % 2 != 0:
        raise ValueError("payload hex must have even length")
    data = bytes.fromhex(payload_hex)
    if len(data) < dlc:
        data = data + bytes([0] * (dlc - len(data)))
    return data[:dlc]


def main():
    args = parse_args()
    if args.dlc < 0 or args.dlc > 8:
        print("dlc must be 0..8", file=sys.stderr)
        sys.exit(1)

    maybe_set_bitrate(args.iface, args.bitrate)

    bus = can.interface.Bus(channel=args.iface, interface="socketcan")
    end_time = time.time() + args.duration
    gap_s = args.gap_us / 1_000_000.0

    sent = 0
    while time.time() < end_time:
        if args.random_id:
            can_id = random.randint(0, 0x7FF if not args.extended else 0x1FFFFFFF)
        else:
            can_id = args.can_id

        data = parse_payload(args.dlc, args.payload)
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=args.extended)
        try:
            bus.send(msg, timeout=0.001)
            sent += 1
        except can.CanError:
            pass
        if gap_s > 0:
            time.sleep(gap_s)

    print(f"Sent {sent} frames on {args.iface}")


if __name__ == "__main__":
    main()
