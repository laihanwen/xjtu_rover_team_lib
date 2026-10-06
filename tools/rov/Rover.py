"""PC joystick sender. Default prints frames; --send explicitly enables networking."""
import argparse
import socket
import time
from Re_control import RemoteControl, load_mapping


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.137.150")
    parser.add_argument("--port", type=int, default=8888)
    parser.add_argument("--dial-min", type=float, default=-0.5)
    parser.add_argument("--dial-max", type=float, default=-0.1)
    parser.add_argument("--send", action="store_true")
    parser.add_argument("--mapping-config", help="explicit, device-specific mapping JSON")
    args = parser.parse_args()
    controller = RemoteControl(args.dial_min, args.dial_max,
                               load_mapping(args.mapping_config) if args.mapping_config else None)
    connection = None
    try:
        while True:
            frame = controller.read()
            if frame is None:
                if connection:
                    connection.close()
                    connection = None
                time.sleep(0.1)
                continue
            if not args.send:
                print(frame.hex(" "))
                time.sleep(0.1)
                continue
            try:
                if connection is None:
                    connection = socket.create_connection((args.host, args.port), timeout=0.2)
                    connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                connection.sendall(frame)
                time.sleep(0.02)
            except OSError as error:
                print(f"Link closed: {error}")
                if connection:
                    connection.close()
                connection = None
                time.sleep(1)
    finally:
        if connection:
            connection.close()


if __name__ == "__main__":
    main()
