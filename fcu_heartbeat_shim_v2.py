#!/usr/bin/env python3
import argparse, socket, time, threading
from pymavlink.dialects.v20 import common as mavlink2  # no custom dialects needed

def hb_sender(out_host, out_port, sysid, compid, autopilot, rate_hz, stop_flag):
    mav = mavlink2.MAVLink(None)
    mav.srcSystem = sysid
    mav.srcComponent = compid
    period = 1.0 / max(0.01, rate_hz)
    base_mode = 0
    custom_mode = 0
    system_status = 4  # MAV_STATE_ACTIVE
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    while not stop_flag["stop"]:
        try:
            pkt = mav.heartbeat_encode(
                mavlink2.MAV_TYPE_GENERIC,   # type
                autopilot,                   # autopilot (0=GENERIC, 3=APM, 12=PX4)
                base_mode,
                custom_mode,
                system_status,
            )
            sock.sendto(pkt.pack(mav), (out_host, out_port))
        except Exception:
            pass
        time.sleep(period)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="in_addr",  default="127.0.0.1:14560")
    ap.add_argument("--out", dest="out_addr", default="127.0.0.1:14600")
    ap.add_argument("--sysid", type=int, default=1)
    ap.add_argument("--compid", type=int, default=1)   # AUTOPILOT1
    ap.add_argument("--autopilot", type=int, default=0)  # 0=GENERIC
    ap.add_argument("--rate", type=float, default=1.0)
    args = ap.parse_args()

    in_host, in_port  = args.in_addr.split(":");  in_port  = int(in_port)
    out_host, out_port = args.out_addr.split(":"); out_port = int(out_port)

    # Bind input; forward raw to OUT
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind((in_host, in_port))
    rx.settimeout(0.1)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    stop_flag = {"stop": False}
    t = threading.Thread(target=hb_sender, args=(out_host, out_port, args.sysid, args.compid, args.autopilot, args.rate, stop_flag), daemon=True)
    t.start()

    print(f"[shim] {args.in_addr} -> {args.out_addr}; injecting HEARTBEAT sysid={args.sysid} compid={args.compid} autopilot={args.autopilot} at {args.rate} Hz")
    try:
        while True:
            try:
                data, _ = rx.recvfrom(65535)
                tx.sendto(data, (out_host, out_port))
            except socket.timeout:
                continue
    except KeyboardInterrupt:
        pass
    finally:
        stop_flag["stop"] = True
        t.join(timeout=1)

if __name__ == "__main__":
    main()
