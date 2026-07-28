#!/usr/bin/env python3
"""Interactive serial console with raw-log and MARK-CSV capture."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
from pathlib import Path
import threading
import time

import serial


DEFAULT_HEADER = (
    "time_ms,label,m5_pulses,m5_cw_deg,"
    "id6_raw,id6_motor_deg,id6_nominal_mm,id6_state,"
    "id7_raw,id7_motor_deg,id7_nominal_mm,id7_state"
)


def timestamp() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def build_paths(output_dir: Path) -> tuple[Path, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = datetime.now().strftime("arm_calibration_%Y%m%d_%H%M%S")
    return output_dir / f"{stem}.log", output_dir / f"{stem}.csv"


def run_capture(port: str, baud: int, output_dir: Path) -> int:
    log_path, csv_path = build_paths(output_dir)
    stop_event = threading.Event()
    file_lock = threading.Lock()

    try:
        connection = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=0.1,
            write_timeout=1.0,
            rtscts=False,
            dsrdtr=False,
        )
    except serial.SerialException as exc:
        print(f"Cannot open {port}: {exc}")
        return 2

    with connection, log_path.open("w", encoding="utf-8", newline="") as log_file, csv_path.open(
        "w", encoding="utf-8", newline=""
    ) as csv_file:
        csv_writer = csv.writer(csv_file, lineterminator="\n")
        csv_header_written = False

        def write_log(direction: str, line: str) -> None:
            with file_lock:
                log_file.write(f"[{timestamp()}] {direction} {line}\n")
                log_file.flush()

        def receive_loop() -> None:
            nonlocal csv_header_written
            while not stop_event.is_set():
                try:
                    raw = connection.readline()
                except serial.SerialException as exc:
                    write_log("ERROR", f"serial_read: {exc}")
                    stop_event.set()
                    return
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                print(line, flush=True)
                write_log("RX", line)

                if line.startswith("MARK_HEADER,"):
                    header = line[len("MARK_HEADER,") :]
                    with file_lock:
                        csv_writer.writerow(header.split(","))
                        csv_file.flush()
                    csv_header_written = True
                elif line.startswith("MARK,"):
                    values = line.split(",", 1)[1].split(",")
                    with file_lock:
                        if not csv_header_written:
                            csv_writer.writerow(DEFAULT_HEADER.split(","))
                            csv_header_written = True
                        csv_writer.writerow(values)
                        csv_file.flush()

        receiver = threading.Thread(target=receive_loop, name="serial-reader", daemon=True)
        receiver.start()

        print(f"Raw log: {log_path}")
        print(f"MARK CSV: {csv_path}")
        print("Type firmware commands here. Ctrl+C exits; ! remains the emergency stop.")
        try:
            while not stop_event.is_set():
                try:
                    command = input()
                except EOFError:
                    break
                command = command.strip()
                if not command:
                    continue
                payload = (command + "\r\n").encode("ascii", errors="replace")
                try:
                    connection.write(payload)
                    connection.flush()
                except serial.SerialException as exc:
                    write_log("ERROR", f"serial_write: {exc}")
                    break
                write_log("TX", command)
        except KeyboardInterrupt:
            print("\nStopping capture.")
        finally:
            stop_event.set()
            receiver.join(timeout=1.0)
            time.sleep(0.1)

    print(f"Saved: {log_path}")
    print(f"Saved: {csv_path}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM10", help="STM32 debug serial port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "calibration_logs",
        help="directory for .log and .csv files",
    )
    args = parser.parse_args()
    return run_capture(args.port, args.baud, args.output_dir)


if __name__ == "__main__":
    raise SystemExit(main())
