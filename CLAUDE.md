# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A single-command CLI (`fnb58`) that reads measurements from a **FNIRSI FNB58**
USB power meter over **Bluetooth LE** and logs them to CSV. The entire
implementation lives in `src/fnb58/cli.py`; everything else is packaging.

## Commands

```bash
uv sync                 # install deps + the package (editable) into .venv
uv run fnb58 --help     # run the CLI from the working tree
uv build                # build wheel + sdist into dist/
uv tool install .       # install `fnb58` as an isolated global tool (~/.local/bin)
python -m py_compile src/fnb58/cli.py   # quick syntax check

# Runtime modes (require a real meter with Bluetooth enabled in its menu):
uv run fnb58            # discover + log all matching meters to one CSV
uv run fnb58 --scan     # list nearby BLE devices and exit
uv run fnb58 --discover # dump GATT table + raw packets (debugging new firmware)
```

There is **no automated test suite**. The BLE path cannot be exercised without
hardware; verify protocol/parse changes by driving `Decoder`/`CsvSink` directly
with synthetic bytes (see how segments are built: `aa | type | len | payload |
checksum`), and run a real meter for end-to-end confirmation.

## Architecture (the non-obvious parts)

**The BLE protocol is completely different from the device's USB-HID protocol.**
This is the central gotcha — do not reuse USB knowledge:

- USB-HID (the widely-documented baryluk protocol) uses 64-byte fixed frames,
  an `aa 81/82/83` handshake, and ÷100000 scaling. **None of that works over BLE.**
- BLE uses **short 4-byte commands**: `aa 81 00 f4` (request info) then
  `aa 82 00 a7` (start streaming); stop is `aa 84 00 01`. No keep-alive needed.
- GATT: notify char `0000ffe4-…`, write char `0000ffe9-…` (constants at top of
  `cli.py`).
- Data is a **TLV stream**, not fixed frames: `0xAA | type | len | payload[len]
  | checksum`. One BLE notification may contain several concatenated segments,
  and segments may be split across notifications.
- Segment types and scaling: `0x04` V/I/power (÷10000), `0x05` resistance/temp,
  `0x06` D+/D−, `0x08` energy/capacity, `0x03` device info. Protocol source:
  https://github.com/zhiyb/FNB58_mqtt_forwarder

**`Decoder`** owns reassembly: it buffers incoming bytes, resyncs on `0xAA`,
waits for a complete segment, dispatches by type, keeps the *latest* value of
each quantity in `self.state`, and fires `on_reading` only when a `0x04`
measurement arrives (merging in the most recent D+/D−, temp, etc.).

**Multi-device design:** `find_devices()` returns *all* name-matching meters;
`run()` opens one shared `CsvSink` and `asyncio.gather`s one `log_device` task
per meter. All workers write to the same CSV, distinguished by the `device`
column (the meter's advertised name). The shared writer needs **no lock**
because bleak dispatches all notification callbacks on the single asyncio
event-loop thread. Per-worker `try/except` + `return_exceptions=True` isolate a
failing meter so it can't abort the others.

## Conventions

- Measurement/log output goes to **stdout**; all status, errors, and device
  info go to **stderr** (so CSV piping stays clean).
- The console-script entry point is `fnb58 = "fnb58.cli:main"` (`pyproject.toml`).
  Build backend is hatchling with a `src/` layout.
