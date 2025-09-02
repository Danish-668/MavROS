#!/usr/bin/env python3
import argparse, socket, time, threading
from typing import Tuple

# We only use pymavlink to SEND a proper heartbeat; forwarding stays raw.
from pymavlink import mavutil

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="in_addr", default="127.0.0.1:14560", help="udp IN host:port")
    ap.add_argument("--out", dest="out_addr", default="127.0.0.1:14600", help="udp OUT host:port (for MAVROS)")
    ap.add_argument("--sysid", type=int, default=1, help="FCU sysid to advertise")
    ap.add_argument("--compid", type=int, default=1, help="FCU compid to advertise (1 = AUTOPILOT1)")
    ap.add_argument("--autopilot", type=int, default=0, help="MAV_AUTOPILOT enum (0=GENERIC, 3=ARDUPILOT, 12=PX4)")
    ap.add_argument("--rate", type=float, default=1.0, help="heartbeat Hz")
    args = ap.parse_args()

    in_host, in_port = args.in_addr.split(":")
    out_host, out_port = args.out_addr.split(":")
    in_port, out_port = int(in_port), int(out_port)

    # Raw UDP sockets: bind IN, forward to OUT
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind((in_host, in_port))
    rx.settimeout(0.1)

    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Pymavlink connection used only to SEND heartbeat frames to OUT
    hb = mavutil.mavlink_connection(
        f"udpout:{out_host}:{out_port}",
        source_system=args.sysid,
        source_component=args.compid,
        dialect="common"  # or "common" if you prefer; heartbeat is in common
    )

    stop = False

    def hb_loop():
        period = 1.0 / max(0.01, args.rate)
        base_mode = 0
        custom_mode = 0
        system_status = 4  # MAV_STATE_ACTIVE
        while not stop:
            try:
                hb.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GENERIC,
                    args.autopilot,
                    base_mode,
                    custom_mode,
                    system_status,
                )
            except Exception:
                pass
            time.sleep(period)

    t = threading.Thread(target=hb_loop, daemon=True)
    t.start()

    print(f"[shim] forwarding {in_host}:{in_port} -> {out_host}:{out_port} and injecting FCU HEARTBEAT "
          f"(sysid={args.sysid}, compid={args.compid}, autopilot={args.autopilot}) at {args.rate} Hz")

    try:
        while True:
            try:
                data, _ = rx.recvfrom(4096)
            except socket.timeout:
                continue
            # forward unmodified
            tx.sendto(data, (out_host, out_port))
    except KeyboardInterrupt:
        pass
    finally:
        stop = True
        t.join(timeout=1)

if __name__ == "__main__":
    main()
