#!/usr/bin/env python3
"""
Read voltage / current / power / D+ / D- / temperature / energy from a
FNIRSI FNB58 USB power meter over Bluetooth Low Energy.

BLE protocol (distinct from the device's USB-HID protocol), per
https://github.com/zhiyb/FNB58_mqtt_forwarder :

  Init (sent to the write characteristic):
      aa 81 00 f4     -- wake / request device info
      aa 82 00 a7     -- start continuous streaming
  (Stop, if you want it:  aa 84 00 01)

  Streaming data arrives on the notify characteristic as a TLV stream.
  One BLE notification may contain several concatenated segments:

      0xAA | type | len | payload[len] | checksum

  Segment types:
      0x03  device info   (model, fw, serial, boot count)
      0x04  measurement   volt=u32/1e4, current=u32/1e4, power=u32/1e4
      0x05  resistance/T  res=u32/1e4, temp=u16/10
      0x06  D+/D-         dp=u16/1e3, dm=u16/1e3
      0x07  low-prec V/I  volt=u16, current=u16 (mV / mA)
      0x08  charge stats  group, energy=u32/1e5, capacity=u32/1e5, time, runtime

Requires:  pip install bleak
"""

import argparse
import asyncio
import csv
import struct
import sys
import time
from datetime import datetime, timezone

from bleak import BleakClient, BleakScanner

# ---------------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------------

SYNC_BYTE = 0xAA

# GATT characteristics (stable across the FNB58 firmware seen so far).
NOTIFY_UUID = "0000ffe4-0000-1000-8000-00805f9b34fb"
WRITE_UUID = "0000ffe9-0000-1000-8000-00805f9b34fb"

# Init / control commands (4 bytes each; last byte is an 8-bit checksum).
CMD_REQUEST_INFO = bytes([0xAA, 0x81, 0x00, 0xF4])
CMD_START_STREAM = bytes([0xAA, 0x82, 0x00, 0xA7])
CMD_STOP_STREAM = bytes([0xAA, 0x84, 0x00, 0x01])

# Names the device may advertise under (case-insensitive substring match).
DEVICE_NAME_HINTS = ("FNB58", "FNB-58", "FNB48", "FNB38", "FNIRSI")


# ---------------------------------------------------------------------------
# TLV stream decoder
# ---------------------------------------------------------------------------

class Decoder:
    """Reassembles BLE notification fragments and decodes the FNB58 TLV
    segments. Keeps the latest reading of each quantity and invokes
    `on_reading` whenever a fresh measurement (type 0x04) arrives."""

    def __init__(self, on_reading):
        self._buf = bytearray()
        self._on_reading = on_reading
        self.state = {
            "voltage_V": 0.0,
            "current_A": 0.0,
            "power_W": 0.0,
            "resistance_ohm": 0.0,
            "temp_C": 0.0,
            "dplus_V": 0.0,
            "dminus_V": 0.0,
            "energy_Wh": 0.0,
            "capacity_Ah": 0.0,
        }
        self.info = None  # populated from a type 0x03 segment

    def feed(self, chunk: bytes):
        self._buf.extend(chunk)
        while True:
            if not self._buf:
                return
            if self._buf[0] != SYNC_BYTE:
                del self._buf[0]            # resync
                continue
            if len(self._buf) < 3:
                return                       # need type + len
            plen = self._buf[2]
            total = 3 + plen + 1             # header(3) + payload + checksum(1)
            if len(self._buf) < total:
                return                       # wait for the rest of the segment
            seg_type = self._buf[1]
            payload = bytes(self._buf[3:3 + plen])
            del self._buf[:total]
            self._handle(seg_type, payload)

    def _handle(self, seg_type: int, pld: bytes):
        s = self.state
        if seg_type == 0x04 and len(pld) == 12:
            s["voltage_V"] = struct.unpack_from("<I", pld, 0)[0] / 10000.0
            s["current_A"] = struct.unpack_from("<I", pld, 4)[0] / 10000.0
            s["power_W"] = struct.unpack_from("<I", pld, 8)[0] / 10000.0
            self._on_reading(dict(s))        # emit a snapshot
        elif seg_type == 0x05 and len(pld) == 7:
            s["resistance_ohm"] = struct.unpack_from("<I", pld, 0)[0] / 10000.0
            s["temp_C"] = struct.unpack_from("<H", pld, 5)[0] / 10.0
        elif seg_type == 0x06 and len(pld) == 6:
            s["dplus_V"] = struct.unpack_from("<H", pld, 0)[0] / 1000.0
            s["dminus_V"] = struct.unpack_from("<H", pld, 2)[0] / 1000.0
        elif seg_type == 0x08 and len(pld) == 17:
            # energy/capacity are reported in Ws / As -> convert to Wh / Ah
            s["energy_Wh"] = struct.unpack_from("<I", pld, 1)[0] / 100000.0 / 3600.0
            s["capacity_Ah"] = struct.unpack_from("<I", pld, 5)[0] / 100000.0 / 3600.0
        elif seg_type == 0x03 and len(pld) == 14:
            self.info = {
                "model": struct.unpack_from("<H", pld, 0)[0],
                "fw_version": struct.unpack_from("<H", pld, 2)[0],
                "serial": struct.unpack_from("<I", pld, 4)[0],
                "boot_count": struct.unpack_from("<I", pld, 8)[0],
            }


# ---------------------------------------------------------------------------
# BLE helpers
# ---------------------------------------------------------------------------

def device_id(dev):
    """Human-readable, unique-per-meter identifier used for the CSV `device`
    column and console tags. The advertised name (e.g. FNB58-051772) already
    carries a serial-like suffix; fall back to the BLE address."""
    return dev.name or dev.address


async def find_devices(address=None, name=None, timeout=10.0):
    """Return all matching FNB58 devices (a list). With --address, returns just
    that one. Raises SystemExit if none are found."""
    if address:
        print(f"Looking for device {address} ...", file=sys.stderr)
        dev = await BleakScanner.find_device_by_address(address, timeout=timeout)
        if dev:
            return [dev]
        raise SystemExit(f"Device with address {address} not found.")

    print("Scanning for FNB58 ...", file=sys.stderr)
    devices = await BleakScanner.discover(timeout=timeout)
    wanted = (name,) if name else DEVICE_NAME_HINTS
    matches = [
        dev for dev in devices
        if any(w.lower() in (dev.name or "").lower() for w in wanted)
    ]
    if not matches:
        raise SystemExit(
            "No FNB58 found. Run with --scan to list all BLE devices, then pass "
            "--address <addr> or --name <name>. Make sure Bluetooth is enabled in "
            "the meter's on-screen menu."
        )
    print(f"Found {len(matches)} device(s): "
          f"{', '.join(device_id(d) for d in matches)}", file=sys.stderr)
    return matches


# ---------------------------------------------------------------------------
# Streaming
# ---------------------------------------------------------------------------

CSV_FIELDS = [
    "epoch", "utc_time", "device", "voltage_V", "current_A", "power_W",
    "dplus_V", "dminus_V", "temp_C", "resistance_ohm",
    "energy_Wh", "capacity_Ah",
]
# Numeric measurement fields (everything after the leading metadata columns).
_MEASUREMENT_FIELDS = CSV_FIELDS[3:]


async def stream(client: BleakClient, dev_id, writer=None, quiet=False, tag=False):
    """Stream one meter. Rows are written to the shared `writer` (may be None),
    each tagged with `dev_id` in the `device` column. When `tag` is set, console
    lines are prefixed with the device id so concurrent output stays readable."""
    prefix = f"[{dev_id}] " if tag else ""

    def on_reading(s):
        t = time.time()
        utc = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
        # bleak dispatches notification callbacks on the asyncio event-loop
        # thread, so the shared writer is only ever touched from one thread.
        if writer:
            row = {"epoch": f"{t:.3f}", "utc_time": utc, "device": dev_id}
            for k in _MEASUREMENT_FIELDS:
                row[k] = f"{s[k]:.5f}"
            writer.writerow(row)
            writer.flush_file()
        if not quiet:
            print(f"{prefix}{utc}  {s['voltage_V']:8.4f} V  {s['current_A']:8.4f} A  "
                  f"{s['power_W']:8.4f} W   D+={s['dplus_V']:.2f} D-={s['dminus_V']:.2f}  "
                  f"{s['temp_C']:5.1f}C  {s['energy_Wh']:.4f} Wh")

    decoder = Decoder(on_reading)

    def notify_cb(_handle, data: bytearray):
        decoder.feed(bytes(data))

    await client.start_notify(NOTIFY_UUID, notify_cb)
    await client.write_gatt_char(WRITE_UUID, CMD_REQUEST_INFO, response=True)
    await asyncio.sleep(0.1)
    await client.write_gatt_char(WRITE_UUID, CMD_START_STREAM, response=True)

    try:
        while client.is_connected:
            await asyncio.sleep(1.0)
            if decoder.info and not getattr(decoder, "_printed_info", False):
                print(f"# {prefix}device: model FNB{decoder.info['model']} "
                      f"fw {decoder.info['fw_version']} "
                      f"sn {decoder.info['serial']}", file=sys.stderr)
                decoder._printed_info = True
    finally:
        try:
            await client.write_gatt_char(WRITE_UUID, CMD_STOP_STREAM, response=True)
            await client.stop_notify(NOTIFY_UUID)
        except Exception:
            pass


class CsvSink:
    """A single shared CSV file written to by all device workers."""

    def __init__(self, path):
        self.path = path
        self._file = open(path, "w", newline="")
        self._writer = csv.DictWriter(self._file, fieldnames=CSV_FIELDS)
        self._writer.writeheader()
        self._file.flush()

    def writerow(self, row):
        self._writer.writerow(row)

    def flush_file(self):
        self._file.flush()

    def close(self):
        self._file.close()


async def log_device(dev, dev_id, sink=None, quiet=False, tag=False):
    """Connect to one meter and stream it. Errors are isolated so that one
    meter disconnecting or failing does not abort the others."""
    try:
        async with BleakClient(dev) as client:
            print(f"Connected to {dev_id} [{dev.address}]", file=sys.stderr)
            await stream(client, dev_id, writer=sink, quiet=quiet, tag=tag)
    except Exception as e:
        print(f"[{dev_id}] error: {e}", file=sys.stderr)


async def discover(client: BleakClient):
    """Dump the GATT table and raw notifications (for debugging new firmware)."""
    print("\n=== GATT services / characteristics ===")
    for service in client.services:
        print(f"[service] {service.uuid}  {service.description}")
        for ch in service.characteristics:
            print(f"    [char] {ch.uuid}  props={','.join(ch.properties)}")
    try:
        print(f"Negotiated MTU: {client.mtu_size} bytes")
    except Exception:
        pass

    def raw_cb(_handle, data: bytearray):
        print("RX", data.hex())

    await client.start_notify(NOTIFY_UUID, raw_cb)
    print("\nSending init; dumping raw packets (Ctrl-C to stop)...\n")
    await client.write_gatt_char(WRITE_UUID, CMD_REQUEST_INFO, response=True)
    await asyncio.sleep(0.1)
    await client.write_gatt_char(WRITE_UUID, CMD_START_STREAM, response=True)
    try:
        while client.is_connected:
            await asyncio.sleep(1.0)
    finally:
        await client.stop_notify(NOTIFY_UUID)


async def run(args):
    if args.scan:
        print("Scanning (5 s)...", file=sys.stderr)
        for dev in await BleakScanner.discover(timeout=5.0):
            print(f"{dev.address}   {dev.name or '<unknown>'}")
        return

    devices = await find_devices(address=args.address, name=args.name)

    # --discover is a single-device debugging mode (uses the first match).
    if args.discover:
        dev = devices[0]
        async with BleakClient(dev) as client:
            print(f"Connected to {device_id(dev)} [{dev.address}]", file=sys.stderr)
            await discover(client)
        return

    sink = None
    if not args.no_file:
        output_path = args.output or datetime.now().strftime("fnb58_%Y%m%d_%H%M%S.csv")
        sink = CsvSink(output_path)
        print(f"Logging to {output_path}", file=sys.stderr)

    tag = len(devices) > 1  # prefix console lines only when multiple meters
    try:
        await asyncio.gather(
            *(log_device(dev, device_id(dev), sink=sink, quiet=args.quiet, tag=tag)
              for dev in devices),
            return_exceptions=True,
        )
    finally:
        if sink:
            sink.close()


def main():
    p = argparse.ArgumentParser(description="Read FNIRSI FNB58 over Bluetooth (BLE).")
    p.add_argument("--scan", action="store_true", help="List nearby BLE devices and exit.")
    p.add_argument("--discover", action="store_true",
                   help="Dump GATT table and raw packets (single device, debugging).")
    p.add_argument("--address", help="Connect directly to this BLE address/UUID.")
    p.add_argument("--name", help="Match device by advertised name substring.")
    p.add_argument("-o", "--output", metavar="FILE",
                   help="CSV file to write (default: fnb58_<timestamp>.csv).")
    p.add_argument("--no-file", action="store_true",
                   help="Do not write a CSV file (console output only).")
    p.add_argument("--quiet", action="store_true",
                   help="Suppress console output (log to CSV only).")
    args = p.parse_args()

    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        print("\nStopped.", file=sys.stderr)


if __name__ == "__main__":
    main()
