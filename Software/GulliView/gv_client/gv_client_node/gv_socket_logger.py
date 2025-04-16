#!/usr/bin/env python3
import argparse
import csv
import json
import pathlib
import signal
import struct
import threading
from socketserver import BaseRequestHandler, UDPServer
from typing import Union, Optional, Generator

from gv_client_node.gullivutil import parse_packet

class Shutdown(Exception):
    """Raise to signal shutdown"""
    pass


def _sigterm_handler(_signo, _stack_frame):
    raise Shutdown


def unpack_data(buf: bytearray, start: int) -> int:
    return struct.unpack('>I', buf[start:start+4])[0]


class GulliViewPacketHandler(BaseRequestHandler):
    start_event: threading.Event = None
    listen_tag_id: Union[str, int] = None
    _file_name_gen = None
    _csv_file = None
    _csv_writer = None
    _writer_lock = threading.Lock()

    def handle(self):
        with self._writer_lock:
            if not self.start_event.is_set():
                return

            recv_buf = bytearray(self.request[0])
            packet = parse_packet(recv_buf)

            for det in packet.detections:
                if self.listen_tag_id != "all" and det.tag_id != self.listen_tag_id:
                    continue
                self._csv_writer.writerow([
                    packet.header.timestamp,
                    det.tag_id,
                    det.camera_id,
                    det.x / 1000,
                    det.y / 1000,
                    det.theta
                ])

    @staticmethod
    def _file_name_generator(initial_name: str) -> Generator[str, None, None]:
        initial = pathlib.Path(initial_name)
        count = 0
        while True:
            yield str(initial.with_stem(f'{initial.stem}_{count}'))
            count += 1

    @classmethod
    def rotate_log_file(cls, name: Optional[str] = None):
        with cls._writer_lock:
            if cls._csv_file is None:
                cls._file_name_gen = cls._file_name_generator(name)

            if cls._csv_file is not None:
                cls._csv_file.close()

            next_file = next(cls._file_name_gen)
            print(f"[+] Rotating logfile to '{next_file}'")
            cls._csv_file = open(next_file, "w", newline='')
            cls._csv_writer = csv.writer(cls._csv_file, dialect='excel')
            cls._csv_writer.writerow(["time", "tag", "camera", "x", "y", "theta"])

    @classmethod
    def shutdown(cls):
        with cls._writer_lock:
            if cls._csv_file:
                cls._csv_file.close()
                print("[+] Logfile closed")


class ControllerPacketHandler(BaseRequestHandler):
    start_event: threading.Event = None

    def handle(self):
        msg = json.loads(self.request[0])
        if msg['type'] == 'start' and not self.start_event.is_set():
            print("[*] Received start from controller")
            self.start_event.set()


class IntersectionPacketHandler(BaseRequestHandler):
    start_event: threading.Event = None

    def handle(self):
        msg = json.loads(self.request[0])
        if not self.start_event.is_set():
            return

        if msg["MSGTYPE"] == "EXIT":
            print("[*] Received coordination EXIT, stopping logging")
            self.start_event.clear()
            GulliViewPacketHandler.rotate_log_file()


def run_server(server: UDPServer):
    thread_name = threading.current_thread().name
    server_address = ':'.join(map(str, server.server_address))
    print(f"[+] Starting {thread_name} server on {server_address}")
    with server:
        server.serve_forever()


def main():
    parser = argparse.ArgumentParser("gv_logger")
    parser.add_argument("-a", "--addr", default="0.0.0.0")
    parser.add_argument("-p", "--port", type=int, default=2121)
    parser.add_argument("-c", "--coordport", type=int, default=2323)
    parser.add_argument("-o", "--controlport", type=int, default=2424)
    parser.add_argument("-t", "--tag", default="all")
    parser.add_argument("-f", "--file", default="gv.csv")
    args = parser.parse_args()

    signal.signal(signal.SIGTERM, _sigterm_handler)
    signal.signal(signal.SIGINT, _sigterm_handler)

    start_logging_event = threading.Event()

    # Setup control
    ControllerPacketHandler.start_event = start_logging_event
    control_server = UDPServer((args.addr, args.controlport), ControllerPacketHandler)
    control_thread = threading.Thread(target=run_server, name='control', args=[control_server])

    # Setup coordination
    IntersectionPacketHandler.start_event = start_logging_event
    coordination_server = UDPServer((args.addr, args.coordport), IntersectionPacketHandler)
    coordination_thread = threading.Thread(target=run_server, name='coordination', args=[coordination_server])

    # Setup datalogger
    print(f"[+] Setting up datalogger, logging to '{args.file}', filtering tag: {args.tag}")
    GulliViewPacketHandler.start_event = start_logging_event
    GulliViewPacketHandler.listen_tag_id = args.tag
    GulliViewPacketHandler.rotate_log_file(name=args.file)
    logging_server = UDPServer((args.addr, args.port), GulliViewPacketHandler)
    logging_thread = threading.Thread(target=run_server, name='logging', args=[logging_server])

    try:
        logging_thread.start()
        control_thread.start()
        coordination_thread.start()

        signal.pause()
    except Shutdown:
        pass
    finally:
        print("\n[!] Received shutdown, cleaning up...")

        logging_server.shutdown()
        GulliViewPacketHandler.shutdown()
        print("[+] Logging server shutdown")

        control_server.shutdown()
        print("[+] Control server shutdown")

        coordination_server.shutdown()
        print("[+] Coordination server shutdown")

        print("[✓] Clean up complete, exiting.")


if __name__ == "__main__":
    main()
