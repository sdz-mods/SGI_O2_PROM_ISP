#!/usr/bin/env python3
"""
O2_FLASH_PROM.py - Host-side tool for O2 PROM ISP firmware

Usage:
  O2_FLASH_PROM.py --port <port> --flash <file.bin> [--verify]
  O2_FLASH_PROM.py --port <port> --dump <file.bin>
  O2_FLASH_PROM.py --port <port> --version
  O2_FLASH_PROM.py --port <port> --boot
  O2_FLASH_PROM.py --port <port> --setenv <key> <value>
  O2_FLASH_PROM.py --port <port> --unsetenv <key>
  O2_FLASH_PROM.py --port <port> --resetenv

  --port   Serial port: e.g. COM32 (Windows) or /dev/ttyUSB0 (Linux)

Requires: pyserial  (pip install pyserial)
"""

import argparse
import sys
import os
import serial
import serial.tools.list_ports
import time
import struct

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
    flash_firmware(ser, data)

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
    print(f"Dumping {FLASH_SIZE:,} bytes to: {filename}")
    
    # Dump the firmware
    received = dump_firmware(ser)
    
    # Save to file
    with open(filename, "wb") as f:
        f.write(received)
    print(f"Dump complete. Saved {len(received):,} bytes to '{filename}'.")


def cmd_setenv(ser: serial.Serial, key: str, value: str) -> None:
    """Set an environment variable in the PROM."""
    # Step 1: Dump the current firmware
    received = dump_firmware(ser)
    
    # Step 2: Find and validate the "env" segment
    env_segment, env_start, env_end, header, offset = find_and_validate_segment(received, "env")

    # Step 3: Parse the env segment
    env_vars = parse_env_vars(env_segment)

    # Step 4: Update or insert the new key-value pair
    env_vars[key] = value
    print(f"Environment variable set: {key}={value}")

    # Step 5: Rebuild and flash the env segment
    build_and_flash_env_segment(ser, received, env_vars, env_start, env_end)


def cmd_unsetenv(ser: serial.Serial, key: str) -> None:
    """Remove an environment variable from the PROM."""
    # Step 1: Dump the current firmware
    received = dump_firmware(ser)
    
    # Step 2: Find and validate the "env" segment
    env_segment, env_start, env_end, header, offset = find_and_validate_segment(received, "env")
    
    # Step 3: Parse the env segment
    env_vars = parse_env_vars(env_segment)
    
    # Step 4: Remove the key-value pair
    if key in env_vars:
        del env_vars[key]
        print(f"Environment variable removed: {key}")
    else:
        print(f"Environment variable not found: {key}")
        return
    
    # Step 5: Rebuild and flash the env segment
    build_and_flash_env_segment(ser, received, env_vars, env_start, env_end)


def cmd_resetenv(ser: serial.Serial) -> None:
    """Reset (empty) the environment variables table."""
    # Step 1: Dump the current firmware
    received = dump_firmware(ser)
    
    # Step 2: Find and validate the "env" segment
    env_segment, env_start, env_end, header, offset = find_and_validate_segment(received, "env")
    
    # Step 3: Clear all environment variables
    env_vars = {}
    print("Environment variables table reset (emptied)")
    
    # Step 4: Rebuild and flash the env segment
    build_and_flash_env_segment(ser, received, env_vars, env_start, env_end)


# ─────────────────────────────────────────────────────────────────────────────
# Helper functions
# ─────────────────────────────────────────────────────────────────────────────

def flash_firmware(ser: serial.Serial, data: bytes) -> None:
    """Flash data to the device."""
    total_chunks = FLASH_SIZE // CHUNK_SIZE
    
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


def dump_firmware(ser: serial.Serial) -> bytes:
    """Dump the entire firmware from the device.
    
    Returns:
        bytes: The firmware data.
    """
    print("\nDumping...")
    send_cmd(ser, "DUMP")
    resp = read_line(ser)
    if not resp.startswith("SIZE "):
        die(f"Unexpected response to DUMP: {resp}")
    declared_size = int(resp.split()[1])
    
    # Check size
    if declared_size != FLASH_SIZE:
        print(f"Warning: firmware reports {declared_size} bytes, expected {FLASH_SIZE}")
    
    # Read the entire flash
    total_chunks = FLASH_SIZE // CHUNK_SIZE
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
    
    return bytes(received)


def find_and_validate_segment(firmware: bytes, seg_name: str) -> tuple:
    """Find and validate a segment in the firmware.
    
    Args:
        firmware: The firmware data.
        seg_name: The name of the segment to find.
    
    Returns:
        tuple: (segment_data, seg_start, seg_end, header, offset)
    """
    
    # Constants for the FlashSegment structure
    SEG_MAGIC = b"SHDR"
    SEG_HEADER_SIZE = 0x40  # Size of the FlashSegment header
    
    # Iterate through the firmware in 0x100 page boundaries
    offset = 0
    segment = None
    seg_start = 0
    seg_end = 0
    header = None
    
    while offset < len(firmware):
        # Check if this is a valid segment header
        if offset + SEG_HEADER_SIZE > len(firmware):
            break
        
        # Extract the header
        header = firmware[offset:offset + SEG_HEADER_SIZE]
        
        # Check magic
        magic = header[8:12]
        if magic != SEG_MAGIC:
            # Skip to the next 0x100 boundary
            offset = (offset // 0x100 + 1) * 0x100
            continue
        
        # Parse the header fields (big-endian)
        seg_len = struct.unpack('>i', header[12:16])[0]
        name_len = header[16]
        seg_type = header[18]
        
        # Extract the segment name
        name_start = 20
        name = header[name_start:name_start + name_len].decode('ascii', errors='replace').strip()
        
        # Debug: Print segment info
        print(f"Found segment: name='{name}', len={seg_len}, type={seg_type}")
        
        # Check if this is the desired segment
        if name == seg_name:
            # Validate the header checksum (big-endian)
            xsum = 0
            for i in range(0, SEG_HEADER_SIZE, 4):
                word = struct.unpack('>i', header[i:i+4])[0]
                xsum = (xsum + word) & 0xffffffff
            
            # The checksum should be 0
            if xsum != 0:
                die(f"Invalid checksum for segment '{name}' at offset {offset}")
            
            seg_start = offset + SEG_HEADER_SIZE
            seg_end = offset + seg_len
            segment = firmware[seg_start:seg_end]
            
            # Validate the segment body checksum
            xsum = 0
            for i in range(0, len(segment), 4):
                word = struct.unpack('>i', segment[i:i+4])[0]
                xsum = (xsum + word) & 0xffffffff
            
            if xsum != 0:
                die(f"Invalid body checksum for segment '{name}' at offset {offset}")
            
            break
        
        # Move to the next segment (segments are aligned to 0x100 boundaries)
        offset = (offset // 0x100 + 1) * 0x100
    
    if segment is None:
        die(f"Could not find '{seg_name}' segment in the firmware")
    
    return segment, seg_start, seg_end, header, offset


def parse_env_vars(env_segment: bytes) -> dict:
    """Parse environment variables from the env segment.
    
    Returns:
        dict: A dictionary of environment variables (key=value pairs).
    """
    env_vars = {}
    
    # Split the env segment into null-terminated strings
    offset = 0
    while offset < len(env_segment):
        # Find the end of the current string
        end = env_segment.find(b'\x00', offset)
        if end == -1:
            break
        
        # Extract the key=value pair
        pair = env_segment[offset:end].decode('ascii', errors='replace')
        
        # Split into key and value
        if '=' in pair:
            k, v = pair.split('=', 1)
            env_vars[k.strip()] = v.strip()
        
        # Move to the next string
        offset = end + 1
    
    return env_vars


def build_and_flash_env_segment(ser: serial.Serial, firmware: bytes, env_vars: dict, env_start: int, env_end: int) -> None:
    """Build the new env segment and flash the updated firmware."""
    # Rebuild the env segment (null-terminated strings)
    new_env_parts = []
    for k, v in env_vars.items():
        pair = f"{k}={v}"
        new_env_parts.append(pair.encode('ascii'))
        new_env_parts.append(b'\x00')  # Null-terminate each pair
    
    new_env_segment = b''.join(new_env_parts)
    
    # Pad the new env segment to the same length as the old one (excluding the checksum)
    env_data_len = (env_end - env_start) - 4  # Last 4 bytes are the checksum
    if len(new_env_segment) > env_data_len:
        die("New environment segment is too large")
    new_env_segment += b'\x00' * (env_data_len - len(new_env_segment))
    
    # Calculate the checksum for the new env segment data (big-endian)
    xsum = 0
    # Sum all 32-bit words in the env segment data (excluding the checksum)
    for i in range(0, len(new_env_segment), 4):
        word = struct.unpack('>i', new_env_segment[i:i+4])[0]
        xsum = (xsum + word) & 0xffffffff
    
    # Append the checksum (negative of the sum, big-endian)
    new_env_segment += struct.pack('>I', (-xsum) & 0xffffffff)
    
    # Rebuild the firmware with the new env segment
    new_firmware = firmware[:env_start] + new_env_segment + firmware[env_end:]
    
    # Flash the new firmware
    flash_firmware(ser, new_firmware)


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
  %(prog)s --port COM32 --setenv var_name var_value
  %(prog)s --port COM32 --unsetenv var_name
  %(prog)s --port COM32 --resetenv
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
    parser.add_argument(
        "--setenv",
        nargs=2,
        metavar=("KEY", "VALUE"),
        help="Set an environment variable in the firmware",
    )
    parser.add_argument(
        "--unsetenv",
        metavar="KEY",
        help="Remove an environment variable from the firmware",
    )
    parser.add_argument(
        "--resetenv",
        action="store_true",
        help="Reset (empty) the environment variables table",
    )
    args = parser.parse_args()

    # At least one action required
    if not (args.flash or args.dump or args.version or args.boot or args.setenv or args.unsetenv or args.resetenv):
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
        elif args.setenv:
            cmd_setenv(ser, args.setenv[0], args.setenv[1])
        elif args.unsetenv:
            cmd_unsetenv(ser, args.unsetenv)
        elif args.resetenv:
            cmd_resetenv(ser)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
