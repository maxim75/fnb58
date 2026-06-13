# fnb58

Command-line tool to read **voltage, current, power, D+/D−, temperature,
resistance, energy and capacity** from a [FNIRSI FNB58](https://www.fnirsi.com/)
USB power meter over **Bluetooth (BLE)**, with live console output and CSV logging.

Works on macOS, Linux and Windows (via [bleak](https://github.com/hbldh/bleak)).

## Install

With [`uv`](https://docs.astral.sh/uv/) (recommended — isolated, on your PATH):

```bash
uv tool install fnb58            # from PyPI
uv tool install .                # from a local checkout
```

Or with `pipx`:

```bash
pipx install fnb58
```

Or plain `pip`:

```bash
pip install fnb58
```

## Usage

Enable Bluetooth in the FNB58's on-screen menu, then:

```bash
fnb58                       # auto-discover, stream to console + timestamped CSV
fnb58 --scan                # list nearby BLE devices and exit
fnb58 -o run1.csv           # choose the CSV filename
fnb58 -o run1.csv --quiet   # log to CSV only, no console output
fnb58 --no-file             # console only, no CSV
fnb58 --address <UUID>      # connect to a specific device
fnb58 --discover            # dump GATT table + raw packets (debugging)
```

Console output:

```
2026-06-13T12:25:31.004+00:00    5.0123 V    0.4988 A    2.4998 W   D+=2.70 D-=2.70   28.3C  0.0021 Wh
```

CSV columns (one row per measurement frame, flushed immediately, UTC timestamps):

```
epoch,utc_time,device,voltage_V,current_A,power_W,dplus_V,dminus_V,temp_C,resistance_ohm,energy_Wh,capacity_Ah
```

### Multiple meters

If several FNB58s are in range, **all** matching meters are logged at once into
**one** CSV file. The `device` column (the meter's advertised name, e.g.
`FNB58-051772`) tells you which row came from which meter, and console lines are
prefixed with `[<device>]`:

```
[FNB58-051772] 2026-06-13T12:25:31.004+00:00    5.0123 V  ...
[FNB58-0517A0] 2026-06-13T12:25:31.020+00:00    9.0001 V  ...
```

```
epoch,utc_time,device,voltage_V,...
1781742331.004,2026-06-13T12:25:31.004+00:00,FNB58-051772,5.01230,...
1781742331.020,2026-06-13T12:25:31.020+00:00,FNB58-0517A0,9.00010,...
```

Use `--name` or `--address` to narrow down to a specific meter. A meter that
drops out is logged to stderr but does not stop the others.

## Notes

- On macOS, grant your terminal Bluetooth permission in
  *System Settings → Privacy & Security → Bluetooth*, and note that BLE
  addresses are reported as UUIDs (use `--scan` to find yours).
- The BLE protocol differs from the device's USB-HID protocol; this tool speaks
  BLE only. Protocol credit:
  [zhiyb/FNB58_mqtt_forwarder](https://github.com/zhiyb/FNB58_mqtt_forwarder)
  and [baryluk/fnirsi-usb-power-data-logger](https://github.com/baryluk/fnirsi-usb-power-data-logger).
