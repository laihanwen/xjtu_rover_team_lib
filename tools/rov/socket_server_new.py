"""Legacy TCP -> UART2 RC relay; does not issue ARM or generate Pi heartbeat."""
import argparse
import socket
import time


class FrameParser:
    def __init__(self):
        self.buffer = bytearray()

    def push(self, data):
        self.buffer.extend(data)
        latest = None
        while self.buffer:
            if self.buffer[0] != 0xA5:
                del self.buffer[0]
                continue
            if len(self.buffer) < 11:
                break
            frame = bytes(self.buffer[:11])
            del self.buffer[:11]
            if frame[6] <= 2 and frame[7] <= 2 and all(x <= 1 for x in frame[8:]):
                latest = frame
        return latest


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8888)
    parser.add_argument("--device", default="/dev/serial0")
    parser.add_argument("--forward", action="store_true")
    args = parser.parse_args()
    uart = None
    if args.forward:
        import serial
        uart = serial.Serial(args.device, 115200, timeout=0, write_timeout=0.1)
    try:
        with socket.socket() as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((args.bind, args.port))
            server.listen(1)
            while True:
                conn, _ = server.accept()
                with conn:
                    conn.settimeout(0.1)
                    frames = FrameParser()
                    last_frame = time.monotonic()
                    try:
                        while True:
                            data = conn.recv(4096)
                            if not data:
                                break
                            if time.monotonic() - last_frame > 0.2:
                                break
                            frame = frames.push(data)
                            if frame is None:
                                continue
                            last_frame = time.monotonic()
                            if uart:
                                if uart.write(frame) != len(frame):
                                    raise OSError("incomplete UART write")
                            else:
                                print(frame.hex(" "))
                    except OSError as error:
                        print(f"Client closed: {error}")
                # Silence lets the STM32 RC timeout disarm. Never replay the last frame.
    finally:
        if uart:
            uart.close()


if __name__ == "__main__":
    main()
