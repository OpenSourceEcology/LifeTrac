"""Unit tests for the bridge's append-only JSONL audit log."""

from __future__ import annotations

import json
import os
import sys
import tempfile
import threading
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from audit_log import AuditLog


class AuditLogTests(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.path = os.path.join(self._tmp.name, "audit.jsonl")

    def tearDown(self) -> None:
        self._tmp.cleanup()

    def _read_lines(self) -> list[dict]:
        with open(self.path, "r", encoding="utf-8") as fp:
            return [json.loads(line) for line in fp if line.strip()]

    def test_records_event_and_extra_fields(self) -> None:
        log = AuditLog(self.path)
        try:
            log.record("rx", source_id=0x01, frame_type=0x10, seq=42)
        finally:
            log.close()
        rows = self._read_lines()
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["event"], "rx")
        self.assertEqual(rows[0]["source_id"], 0x01)
        self.assertEqual(rows[0]["seq"], 42)
        self.assertIn("ts", rows[0])
        self.assertIsInstance(rows[0]["ts"], (int, float))

    def test_bytes_payload_renders_as_hex(self) -> None:
        log = AuditLog(self.path)
        try:
            log.record("snapshot", payload=b"\xde\xad\xbe\xef")
        finally:
            log.close()
        row = self._read_lines()[0]
        self.assertEqual(row["payload"], "deadbeef")

    def test_thread_safe_writes_do_not_interleave(self) -> None:
        log = AuditLog(self.path)
        try:
            barrier = threading.Barrier(8)
            def writer(tid: int) -> None:
                barrier.wait()
                for i in range(50):
                    log.record("burst", tid=tid, i=i)
            threads = [threading.Thread(target=writer, args=(t,)) for t in range(8)]
            for t in threads: t.start()
            for t in threads: t.join()
        finally:
            log.close()
        rows = self._read_lines()
        # Every line must parse and carry both fields — i.e. nothing was clipped
        # by an interleaved write.
        self.assertEqual(len(rows), 8 * 50)
        for row in rows:
            self.assertEqual(row["event"], "burst")
            self.assertIn("tid", row)
            self.assertIn("i", row)

    def test_size_based_rotation(self) -> None:
        # Tiny rotate threshold so we cross it quickly.
        log = AuditLog(self.path, rotate_bytes=512, keep_rotations=3)
        try:
            for i in range(200):
                log.record("pad", filler="x" * 50, i=i)
        finally:
            log.close()
        # We should have at minimum the live file plus one rotation.
        self.assertTrue(os.path.exists(self.path))
        self.assertTrue(os.path.exists(self.path + ".1"))
        # All rotated files (live + .1[.2[.3]]) must contain valid JSONL.
        for suffix in ("", ".1", ".2", ".3"):
            p = self.path + suffix
            if not os.path.exists(p): continue
            with open(p, "r", encoding="utf-8") as fp:
                for line in fp:
                    if line.strip():
                        json.loads(line)   # raises if malformed

    def test_open_failure_degrades_gracefully(self) -> None:
        # Path under a non-existent dir whose parent exists but is read-only-ish:
        # we use an unreachable path on Windows + POSIX by pointing at a NUL file.
        bad = os.path.join(self._tmp.name, "no", "such", "dir", "audit.jsonl")
        # The constructor is allowed to either succeed (creates intermediate
        # dirs) or fail silently. record() must never raise either way.
        log = AuditLog(bad)
        try:
            log.record("safe", value=1)   # must not raise
        finally:
            log.close()


if __name__ == "__main__":
    unittest.main()
