#!/usr/bin/env python3
"""Offline tests: need NO CAN interface. Run: python3 tests/test_offline.py"""
import csv, os, sys, tempfile, time, unittest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import can_common as c
from vehicle_ecu import VehicleModel
from logger_ecu import frame_to_row, HEADER
from dashboard_ecu import DashboardState


class Codec(unittest.TestCase):
    def test_speed_roundtrip(self):
        for v in (0, 0.1, 65.4, 120):
            self.assertAlmostEqual(c.decode_speed(c.encode_speed(v)), v, places=1)

    def test_rpm_temp_roundtrip(self):
        for v in (800, 2450, 5000):
            self.assertEqual(c.decode_rpm(c.encode_rpm(v)), v)
        for v in (20, 88, 120):
            self.assertEqual(c.decode_temp(c.encode_temp(v)), v)

    def test_clamping(self):
        self.assertEqual(c.decode_speed(c.encode_speed(500)), 120)
        self.assertEqual(c.decode_rpm(c.encode_rpm(10)), 800)
        self.assertEqual(c.decode_temp(c.encode_temp(-5)), 20)

    def test_lengths(self):
        self.assertEqual(len(c.encode_speed(1)), 2)
        self.assertEqual(len(c.encode_rpm(1000)), 2)
        self.assertEqual(len(c.encode_temp(50)), 1)
        self.assertEqual(len(c.encode_fd_vehicle(1, 1000, 50, 1, 0)), 16)

    def test_fd_roundtrip(self):
        d = c.decode_fd_vehicle(c.encode_fd_vehicle(65.5, 2450, 88, 4, 123456))
        self.assertEqual((d["speed"], d["rpm"], d["temp"], d["gear"], d["odometer_m"]),
                         (65.5, 2450.0, 88.0, 4, 123456))


class Frames(unittest.TestCase):
    def test_sizes(self):
        self.assertEqual(c.CLASSIC_SIZE, 16)
        self.assertEqual(c.FD_SIZE, 72)

    def test_classic_std(self):
        f = c.unpack_frame(c.pack_frame(0x100, b"\x01\x02"))
        self.assertEqual((f.can_id, f.extended, f.fd, f.length, f.data), (0x100, False, False, 2, b"\x01\x02"))

    def test_extended(self):
        f = c.unpack_frame(c.pack_frame(0x18FEF100, b"\xAA" * 8, extended=True))
        self.assertTrue(f.extended)
        self.assertEqual(f.can_id, 0x18FEF100)

    def test_fd(self):
        f = c.unpack_frame(c.pack_frame(0x200, bytes(range(16)), fd=True))
        self.assertTrue(f.fd and f.brs)
        self.assertEqual(f.length, 16)
        self.assertEqual(f.data, bytes(range(16)))

    def test_invalid(self):
        with self.assertRaises(ValueError):
            c.pack_frame(0x100, bytes(9))               # classic > 8
        with self.assertRaises(ValueError):
            c.pack_frame(0x100, bytes(10), fd=True)     # 10 is not a valid FD length

    def test_filter_exact(self):
        cid, mask = c.exact_filter(0x100)
        self.assertEqual((0x100 & mask), cid)
        self.assertNotEqual((0x101 & mask), cid)
        self.assertNotEqual(((0x100 | c.CAN_EFF_FLAG) & mask), cid)   # ext frame must not match


class Model(unittest.TestCase):
    def test_ranges_over_10_minutes(self):
        m = VehicleModel(seed=1)
        smin, smax, rmin, rmax, tmin, tmax = 999, -1, 9999, -1, 999, -1
        for _ in range(60000):                  # 600 s at 10 ms steps
            m.step(0.01)
            smin, smax = min(smin, m.speed), max(smax, m.speed)
            rmin, rmax = min(rmin, m.rpm), max(rmax, m.rpm)
            tmin, tmax = min(tmin, m.temp), max(tmax, m.temp)
        self.assertGreaterEqual(smin, 0); self.assertLessEqual(smax, 120)
        self.assertGreaterEqual(rmin, 800); self.assertLessEqual(rmax, 5000)
        self.assertGreaterEqual(tmin, 20); self.assertLessEqual(tmax, 120)
        self.assertGreater(smax, 80)            # it actually drives
        self.assertGreater(m.temp, 80)          # engine warmed up

    def test_realistic_change_rate(self):
        m = VehicleModel(seed=2)
        prev = m.speed
        for _ in range(5000):
            m.step(0.01)
            self.assertLess(abs(m.speed - prev), 0.2)   # < 20 km/h per second incl. noise
            prev = m.speed


class LoggerAndDashboard(unittest.TestCase):
    def test_csv_row(self):
        f = c.unpack_frame(c.pack_frame(0x100, c.encode_speed(65)))
        row = frame_to_row(f, ts=1700000000.123456)
        self.assertEqual(row[2:], ["0x100", "STD", "CAN", 2, "8a02"])
        self.assertEqual(len(row), len(HEADER))

    def test_csv_file(self):
        path = tempfile.mktemp(suffix=".csv")
        with open(path, "w", newline="") as fh:
            w = csv.writer(fh); w.writerow(HEADER)
            for cid, data in ((0x100, c.encode_speed(10)), (0x3A0, b"\xDE\xAD")):
                w.writerow(frame_to_row(c.unpack_frame(c.pack_frame(cid, data))))
        with open(path) as fh:
            rows = list(csv.DictReader(fh))
        self.assertEqual([r["can_id"] for r in rows], ["0x100", "0x3A0"])
        os.remove(path)

    def test_dashboard_decode_and_unknown(self):
        st = DashboardState("all", 1.0)
        now = time.monotonic()
        st.on_frame(c.unpack_frame(c.pack_frame(0x100, c.encode_speed(65))), now, False)
        st.on_frame(c.unpack_frame(c.pack_frame(0x101, c.encode_rpm(2450))), now, False)
        st.on_frame(c.unpack_frame(c.pack_frame(0x102, c.encode_temp(88))), now, False)
        st.on_frame(c.unpack_frame(c.pack_frame(0x3A0, b"\x01")), now, False)
        out = st.render(now)
        self.assertIn("Speed       : 65 km/h", out)
        self.assertIn("Engine RPM  : 2450 rpm", out)
        self.assertIn("Temperature : 88", out)
        self.assertNotIn("WARNING", out)
        self.assertEqual(st.unknown_ids, {0x3A0: 1})      # counted, never displayed as a signal

    def test_dashboard_offline_and_recovery(self):
        st = DashboardState("all", 1.0)
        t = time.monotonic()
        st.on_frame(c.unpack_frame(c.pack_frame(0x100, c.encode_speed(50))), t, False)
        self.assertFalse(st.is_offline(t + 0.5))
        self.assertTrue(st.is_offline(t + 1.5))
        self.assertIn("WARNING: Vehicle ECU Offline", st.render(t + 1.5))

    def test_dashboard_filter_modes(self):
        st = DashboardState("speed", 1.0)
        self.assertIn("filtered out", st.render(time.monotonic()).split("Engine RPM")[1].split("\n")[0])
        self.assertEqual(DashboardState("rpm", 1.0).watchdog_id, c.ID_RPM)

    def test_dashboard_fd_frame(self):
        st = DashboardState("all", 1.0)
        now = time.monotonic()
        fr = c.unpack_frame(c.pack_frame(0x200, c.encode_fd_vehicle(70, 3000, 90, 4, 10), fd=True))
        st.on_frame(fr, now, False)
        self.assertEqual((st.values["speed"], st.values["rpm"], st.values["temp"]), (70.0, 3000.0, 90.0))
        self.assertEqual(st.counts["fd"], 1)
        self.assertFalse(st.is_offline(now))


if __name__ == "__main__":
    unittest.main(verbosity=2)
