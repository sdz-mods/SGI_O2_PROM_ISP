#!/usr/bin/env python3
"""
O2_FLASH_PROM.py - Host-side tool for O2 PROM ISP firmware

Usage:
  O2_FLASH_PROM.py --port <port> --flash <file.bin> [--verify]
  O2_FLASH_PROM.py --port <port> --dump <file.bin>
  O2_FLASH_PROM.py --port <port> --version
  O2_FLASH_PROM.py --port <port> --boot

  --port   Serial port: e.g. COM32 (Windows) or /dev/ttyUSB0 (Linux)

Requires: pyserial  (pip install pyserial)
"""

import argparse
import sys
import os
import serial
import serial.tools.list_ports
import time

# ─────────────────────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────────────────────

FLASH_SIZE  = 512 * 1024   # AT29C040A: 512 KB
CHUNK_SIZE  = 4096         # Must match firmware CHUNK_SIZE
BAUD_RATE   = 115200       # Ignored for USB CDC, but required by pyserial
CMD_TIMEOUT = 10           # Seconds to wait for a text response line
DATA_TIMEOUT = 60          # Seconds allowed for a full chunk transfer

# ─────────────────────────────────────────────────────────────────────────────
# Serial helpers
# ─────────────────────────────────────────────────────────────────────────────

def open_port(port: str) -> serial.Serial:
    """Open the CDC serial port."""
    try:
        ser = serial.Serial(
            port,
            baudrate=BAUD_RATE,
            timeout=CMD_TIMEOUT,
            write_timeout=DATA_TIMEOUT,
        )
        # Flush any stale data
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        return ser
    except serial.SerialException as e:
        die(f"Cannot open port '{port}': {e}")


def send_cmd(ser: serial.Serial, cmd: str) -> None:
    """Send a text command (adds newline)."""
    ser.write((cmd + "\n").encode())
    ser.flush()


def read_line(ser: serial.Serial) -> str:
    """Read one response line (strips \\r\\n). Raises on timeout."""
    line = ser.readline()
    if not line:
        die("Timeout waiting for response from firmware")
    return line.decode(errors="replace").strip()


def expect_ok(ser: serial.Serial, context: str = "") -> None:
    """Read a line and die if it is not 'OK' (or 'OK ...')."""
    resp = read_line(ser)
    if not resp.startswith("OK"):
        die(f"Unexpected response{' ' + context if context else ''}: {resp}")


# ─────────────────────────────────────────────────────────────────────────────
# Utility
# ─────────────────────────────────────────────────────────────────────────────

def die(msg: str) -> None:
    print(f"ERROR: {msg}", file=sys.stderr)
    sys.exit(1)


def progress(label: str, done: int, total: int) -> None:
    pct = done * 100 // total
    bar_len = 40
    filled = bar_len * done // total
    bar = "#" * filled + "-" * (bar_len - filled)
    print(f"\r{label}: [{bar}] {pct:3d}%", end="", flush=True)


def pad_to_chunk(data: bytes, chunk: int = CHUNK_SIZE) -> bytes:
    """Pad data with 0xFF so its length is a multiple of chunk."""
    rem = len(data) % chunk
    if rem:
        data += b"\xff" * (chunk - rem)
    return data


# ─────────────────────────────────────────────────────────────────────────────
# Command implementations
# ─────────────────────────────────────────────────────────────────────────────

def cmd_version(ser: serial.Serial) -> None:
    send_cmd(ser, "VER")
    resp = read_line(ser)
    if resp.startswith("OK "):
        print(f"Firmware version: {resp[3:]}")
    elif resp == "OK":
        print("Firmware version: (unknown)")
    else:
        die(f"Unexpected response: {resp}")


def cmd_boot(ser: serial.Serial) -> None:
    send_cmd(ser, "BOOT")
    resp = read_line(ser)
    if resp.startswith("OK"):
        print("MCU is rebooting into BOOTSEL (UF2) mode.")
        print("A USB mass-storage drive will appear — copy your .uf2 file there.")
    else:
        die(f"Unexpected response: {resp}")


def cmd_flash(ser: serial.Serial, filename: str, verify: bool = False) -> None:
    # Read file
    if not os.path.isfile(filename):
        die(f"File not found: {filename}")
    with open(filename, "rb") as f:
        raw = f.read()

    if len(raw) > FLASH_SIZE:
        die(f"File size {len(raw):,} bytes exceeds flash size {FLASH_SIZE:,} bytes")

    original_size = len(raw)
    # Always pad to full FLASH_SIZE with 0xFF so the entire chip is overwritten,
    # leaving no residual data from a previous image beyond the file boundary.
    data = raw + b"\xff" * (FLASH_SIZE - len(raw))
    total_chunks = FLASH_SIZE // CHUNK_SIZE   # always 128

    print(f"File : {filename}  ({original_size:,} bytes)")
    print(f"Flash: {FLASH_SIZE:,} bytes  |  Chunks: {total_chunks} × {CHUNK_SIZE} bytes")

    # ── Flash phase ──────────────────────────────────────────────────────────
    print("\nFlashing...")
    send_cmd(ser, f"FLASH {FLASH_SIZE}")
    expect_ok(ser, "after FLASH command")

    for i in range(total_chunks):
        chunk = data[i * CHUNK_SIZE:(i + 1) * CHUNK_SIZE]
        ser.write(chunk)
        ser.flush()

        # Firmware sends "ACK" after programming 16 pages (~192 ms)
        resp = read_line(ser)
        if resp != "ACK":
            print()
            die(f"Expected ACK after chunk {i}, got: {resp}")
        progress("Flashing", i + 1, total_chunks)

    print()
    resp = read_line(ser)
    if resp != "DONE":
        die(f"Flash did not complete cleanly: {resp}")
    print("Flash complete.")

    # ── Verify phase (optional) ──────────────────────────────────────────────
    if verify:
        cmd_verify_only(ser, data, original_size)


def cmd_verify_only(ser: serial.Serial, data: bytes, original_size: int) -> None:
    """Send VERIFY command and compare host data with flash contents."""
    # data is already padded to FLASH_SIZE; verify the full chip
    total_chunks = FLASH_SIZE // CHUNK_SIZE

    print("\nVerifying...")
    send_cmd(ser, f"VERIFY {FLASH_SIZE}")
    expect_ok(ser, "after VERIFY command")

    for i in range(total_chunks):
        chunk = data[i * CHUNK_SIZE:(i + 1) * CHUNK_SIZE]
        ser.write(chunk)
        ser.flush()

        resp = read_line(ser)
        if resp.startswith("ERR"):
            print()
            # ERR <offset>
            parts = resp.split()
            offset_str = parts[1] if len(parts) > 1 else "?"
            die(f"Mismatch at flash offset {offset_str}")
        elif resp != "ACK":
            print()
            die(f"Expected ACK after verify chunk {i}, got: {resp}")
        progress("Verifying", i + 1, total_chunks)

    print()
    resp = read_line(ser)
    if resp != "DONE":
        die(f"Verify did not complete cleanly: {resp}")
    print("Verify passed — flash contents match file.")


def cmd_dump(ser: serial.Serial, filename: str) -> None:
    total_chunks = FLASH_SIZE // CHUNK_SIZE

    print(f"Dumping {FLASH_SIZE:,} bytes to: {filename}")
    send_cmd(ser, "DUMP")

    # Expect "SIZE 524288"
    resp = read_line(ser)
    if not resp.startswith("SIZE "):
        die(f"Unexpected response to DUMP: {resp}")
    declared_size = int(resp.split()[1])
    if declared_size != FLASH_SIZE:
        print(f"Warning: firmware reports {declared_size} bytes, expected {FLASH_SIZE}")

    received = bytearray()
    ser.timeout = DATA_TIMEOUT

    for i in range(total_chunks):
        chunk = ser.read(CHUNK_SIZE)
        if len(chunk) != CHUNK_SIZE:
            die(f"Short read at chunk {i}: got {len(chunk)} bytes")
        received.extend(chunk)

        # Send ACK for next chunk
        ser.write(b"ACK\n")
        ser.flush()
        progress("Dumping", i + 1, total_chunks)

    print()

    ser.timeout = CMD_TIMEOUT
    resp = read_line(ser)
    if resp != "DONE":
        die(f"Dump did not complete cleanly: {resp}")

    with open(filename, "wb") as f:
        f.write(received)
    print(f"Dump complete. Saved {len(received):,} bytes to '{filename}'.")


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="O2 PROM ISP Flash Tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --port COM32 --version
  %(prog)s --port /dev/ttyACM0 --flash rom.bin
  %(prog)s --port COM32 --flash rom.bin --verify
  %(prog)s --port COM32 --dump backup.bin
  %(prog)s --port COM32 --boot
""",
    )

    parser.add_argument(
        "--port",
        required=True,
        help="Serial port (e.g. COM32 on Windows, /dev/ttyACM0 on Linux)",
    )
    parser.add_argument(
        "--flash",
        metavar="FILE",
        help="Binary file to program into flash",
    )
    parser.add_argument(
        "--verify",
        action="store_true",
        help="Read back and verify after --flash",
    )
    parser.add_argument(
        "--dump",
        metavar="FILE",
        help="Read entire flash and save to FILE",
    )
    parser.add_argument(
        "--version",
        action="store_true",
        help="Report firmware version running on the MCU",
    )
    parser.add_argument(
        "--boot",
        action="store_true",
        help="Reboot MCU into BOOTSEL mode for firmware update",
    )
    args = parser.parse_args()

    # At least one action required
    if not (args.flash or args.dump or args.version or args.boot):
        parser.print_help()
        sys.exit(1)

    # --verify without --flash makes no sense
    if args.verify and not args.flash:
        die("--verify requires --flash")

    print(f"Opening port: {args.port}")
    ser = open_port(args.port)

    try:
        if args.version:
            cmd_version(ser)
        elif args.boot:
            cmd_boot(ser)
        elif args.flash:
            cmd_flash(ser, args.flash, verify=args.verify)
        elif args.dump:
            cmd_dump(ser, args.dump)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
