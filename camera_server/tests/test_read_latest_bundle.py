#!/usr/bin/env python3
import importlib.util
import mmap
import pathlib
import struct
import tempfile
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("read_latest_bundle", ROOT / "tools" / "read_latest_bundle.py")
read_latest_bundle = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(read_latest_bundle)


class ReadLatestBundleBoundsTest(unittest.TestCase):
    def make_mmap(self, slot_size: int = 4):
        tmp = tempfile.TemporaryFile()
        tmp.truncate(256)
        mm = mmap.mmap(tmp.fileno(), 256)
        payload_off = 128
        header_off = payload_off - read_latest_bundle.SLOT_HEADER.size
        header = read_latest_bundle.SLOT_HEADER.pack(
            2, 2, 7, 100, 90, 110, 2, 2, 6, slot_size, 1, 0
        )
        mm[header_off:header_off + len(header)] = header
        mm[payload_off:payload_off + 4] = b"test"
        return tmp, mm, payload_off

    def test_reads_valid_slot(self):
        tmp, mm, payload_off = self.make_mmap()
        try:
            data = read_latest_bundle.read_slot(
                mm, {"ring_name": "head.color", "shm_offset": payload_off, "size_bytes": 4}
            )
            self.assertEqual(data, b"test")
        finally:
            mm.close()
            tmp.close()

    def test_rejects_out_of_bounds_bundle_offset(self):
        tmp, mm, _ = self.make_mmap()
        try:
            with self.assertRaisesRegex(RuntimeError, "out of bounds"):
                read_latest_bundle.read_slot(
                    mm, {"ring_name": "head.color", "shm_offset": 250, "size_bytes": 16}
                )
        finally:
            mm.close()
            tmp.close()

    def test_rejects_bundle_size_that_disagrees_with_slot(self):
        tmp, mm, payload_off = self.make_mmap(slot_size=8)
        try:
            with self.assertRaisesRegex(RuntimeError, "does not match"):
                read_latest_bundle.read_slot(
                    mm, {"ring_name": "head.color", "shm_offset": payload_off, "size_bytes": 4}
                )
        finally:
            mm.close()
            tmp.close()


if __name__ == "__main__":
    raise SystemExit(unittest.main())
