"""Deterministically (re)generate golden.pkraw. Usage: python make_golden.py [OUT_PATH]

The JavaScript test must reproduce this file byte-for-byte.
"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))  # tests/ (for the shared golden hex constants)

from test_protocol import IMU_GOLDEN, STATUS_GOLDEN  # noqa: E402

from powerkettle.rawlog import RawRecord, write_rawlog  # noqa: E402

META = {"app": "golden", "v": 1}
RECORDS = [
    RawRecord(2, 1000.0, STATUS_GOLDEN),
    RawRecord(1, 1001.5, IMU_GOLDEN),
    RawRecord(3, 1002.0, "snatch L".encode()),
    RawRecord(4, 1003.0, '{"e":"disconnect"}'.encode()),
]

if __name__ == "__main__":
    write_rawlog(sys.argv[1] if len(sys.argv) > 1 else HERE / "golden.pkraw", META, RECORDS)
