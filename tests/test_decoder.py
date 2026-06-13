"""Tests for the hardware-independent logic: TLV decoding, CSV sink, device id.

The BLE transport itself can't be exercised without a physical meter, but the
Decoder (frame reassembly + segment parsing), CsvSink and device_id helpers are
pure functions of their inputs and are fully covered here.
"""

import csv
import struct
from types import SimpleNamespace

from fnb58.cli import CSV_FIELDS, CsvSink, Decoder, device_id


def seg(seg_type: int, payload: bytes) -> bytes:
    """Build one TLV segment: 0xAA | type | len | payload | checksum.
    The Decoder ignores the checksum, so a zero byte is fine here."""
    return bytes([0xAA, seg_type, len(payload)]) + payload + bytes([0x00])


def seg_measurement(volt, amp, power):  # type 0x04, 12-byte payload
    return seg(0x04, struct.pack("<III", round(volt * 1e4), round(amp * 1e4), round(power * 1e4)))


def seg_temp(res, temp):  # type 0x05, 7-byte payload
    return seg(0x05, struct.pack("<I", round(res * 1e4)) + b"\x00" + struct.pack("<H", round(temp * 10)))


def seg_dpm(dp, dm):  # type 0x06, 6-byte payload
    return seg(0x06, struct.pack("<HHH", round(dp * 1e3), round(dm * 1e3), 0))


def collect(*chunks):
    """Feed chunks into a fresh Decoder and return the emitted readings."""
    out = []
    dec = Decoder(lambda s: out.append(dict(s)))
    for c in chunks:
        dec.feed(c)
    return out


def test_measurement_scaling():
    [r] = collect(seg_measurement(5.0123, 0.4988, 2.4998))
    assert round(r["voltage_V"], 4) == 5.0123
    assert round(r["current_A"], 4) == 0.4988
    assert round(r["power_W"], 4) == 2.4998


def test_only_0x04_emits_a_reading():
    # temp + D+/D- segments update state but must not emit on their own.
    out = collect(seg_temp(0.5, 28.3), seg_dpm(2.7, 2.7))
    assert out == []


def test_latest_state_merged_into_reading():
    [r] = collect(seg_temp(0.5, 28.3), seg_dpm(2.70, 2.71), seg_measurement(5.0, 0.5, 2.5))
    assert round(r["temp_C"], 1) == 28.3
    assert round(r["dplus_V"], 2) == 2.70
    assert round(r["dminus_V"], 2) == 2.71


def test_reassembly_across_fragmented_notifications():
    frame = seg_measurement(9.0, 1.2, 10.8)
    # split mid-segment across two notifications
    out = collect(frame[:5], frame[5:])
    assert len(out) == 1
    assert round(out[0]["voltage_V"], 1) == 9.0


def test_multiple_segments_in_one_notification():
    blob = seg_dpm(2.7, 2.7) + seg_measurement(5.0, 0.5, 2.5) + seg_measurement(5.1, 0.6, 3.06)
    out = collect(blob)
    assert len(out) == 2
    assert round(out[1]["voltage_V"], 1) == 5.1


def test_resync_on_leading_garbage():
    frame = seg_measurement(5.0, 0.5, 2.5)
    out = collect(b"\x00\xff\x12" + frame)  # junk before a valid segment
    assert len(out) == 1
    assert round(out[0]["voltage_V"], 1) == 5.0


def test_device_id_prefers_name_then_address():
    assert device_id(SimpleNamespace(name="FNB58-051772", address="AA:BB")) == "FNB58-051772"
    assert device_id(SimpleNamespace(name=None, address="AA:BB")) == "AA:BB"


def test_csv_sink_writes_header_and_rows(tmp_path):
    path = tmp_path / "out.csv"
    sink = CsvSink(str(path))
    sink.writerow({k: ("DEV-A" if k == "device" else "1") for k in CSV_FIELDS})
    sink.flush_file()
    sink.close()

    rows = list(csv.DictReader(open(path)))
    assert list(rows[0].keys()) == CSV_FIELDS
    assert rows[0]["device"] == "DEV-A"
