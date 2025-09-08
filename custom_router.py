#!/usr/bin/env python3
"""
Simple MAVLink forwarder that passes ALL messages including custom ones
Replaces mavlink-routerd for testing custom messages
"""
import socket
import threading
import time
import yaml
import sys
import os
import argparse

def load_config(config_path="config.yaml"):
    """Load configuration from YAML file"""
    if os.path.exists(config_path):
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)
    return None

def forward_messages(in_port, out_port, host='127.0.0.1'):
    # Create sockets
    in_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    in_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    in_sock.bind((host, in_port))
    in_sock.settimeout(0.1)
    
    out_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    print(f"Custom router: {host}:{in_port} -> {host}:{out_port}")
    print("Forwarding ALL messages including custom ones...")
    
    msg_count = 0
    custom_count = 0
    
    try:
        while True:
            try:
                data, addr = in_sock.recvfrom(65535)
                if data:
                    # Forward immediately
                    out_sock.sendto(data, (host, out_port))
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
    parser = argparse.ArgumentParser(description='Custom MAVLink router')
    parser.add_argument('--config', default='config.yaml', help='Path to config file')
    parser.add_argument('--in-port', type=int, help='Input port (overrides config)')
    parser.add_argument('--out-port', type=int, help='Output port (overrides config)')
    parser.add_argument('--host', help='Host IP (overrides config)')
    args = parser.parse_args()
    
    # Load config
    config = load_config(args.config)
    
    # Set defaults
    in_port = 14550
    out_port = 14555
    host = '127.0.0.1'
    
    # Override with config if available
    if config:
        mavlink_config = config.get('mavlink', {})
        in_port = mavlink_config.get('input_port', in_port)
        out_port = mavlink_config.get('output_port', out_port)
        host = mavlink_config.get('host', host)
        
        print(f"Loaded config from {args.config}")
    
    # Override with command line args if provided
    if args.in_port:
        in_port = args.in_port
    if args.out_port:
        out_port = args.out_port
    if args.host:
        host = args.host
    
    # Start forwarding
    forward_messages(in_port, out_port, host)