#!/usr/bin/env python3
"""
Simple MAVLink forwarder that passes ALL messages including custom ones
Replaces mavlink-routerd for testing custom messages
"""
import socket
import threading
import time

def forward_messages(in_port, out_port):
    # Create sockets
    in_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    in_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    in_sock.bind(('127.0.0.1', in_port))
    in_sock.settimeout(0.1)
    
    out_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    print(f"Custom router: {in_port} -> {out_port}")
    print("Forwarding ALL messages including custom ones...")
    
    msg_count = 0
    custom_count = 0
    
    try:
        while True:
            try:
                data, addr = in_sock.recvfrom(65535)
                if data:
                    # Forward immediately
                    out_sock.sendto(data, ('127.0.0.1', out_port))
                    msg_count += 1
                    
                    # Check for custom message
                    if len(data) >= 12 and data[0] == 0xFD:
                        msg_id = int.from_bytes(data[7:10], 'little')
                        if msg_id >= 42000:
                            custom_count += 1
                            print(f"Forwarded CUSTOM message ID {msg_id} from {addr}")
                        elif msg_id == 30:  # ATTITUDE message
                            print(f"Forwarded ATTITUDE message (ID 30) from {addr}")
                    
                    # Status every 100 messages
                    if msg_count % 100 == 0:
                        print(f"Forwarded {msg_count} msgs ({custom_count} custom)")
                        
            except socket.timeout:
                continue
                
    except KeyboardInterrupt:
        print(f"\nTotal forwarded: {msg_count} messages ({custom_count} custom)")
    finally:
        in_sock.close()
        out_sock.close()

if __name__ == "__main__":
    # Replace mavlink-routerd: 14550 -> 14555
    forward_messages(14550, 14555)