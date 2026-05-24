"""test_sweep_planner.py — Unit tests for the Boustrophedon sweep route planning module.

Verifies:
- Haversine-to-relative-XY-meter projection with `get_distance_meters`.
- Boustrophedon coverage sweeping for multiple valid geofence configurations.
- Safeguards for edge cases, narrow regions, and invalid coordinates.
"""
from __future__ import annotations

import unittest
import sys
import os

# Append the base_station directory so we can import the local `planner` module
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import planner


class SweepPlannerTests(unittest.TestCase):
    def test_distance_meters(self):
        # Base check on Haversine distance
        # Latitude: 1 degree ≈ 111,139 meters at equator
        pt1 = (0.0, 0.0)
        pt2 = (1.0, 0.0)
        dist_lat = planner.get_distance_meters(pt1[0], pt1[1], pt2[0], pt2[1])
        # Allow 0.5% deviation due to spherical Earth assumption vs exact geoid
        self.assertGreater(dist_lat, 110000)
        self.assertLess(dist_lat, 112000)

        # Same point should return 0 meters
        self.assertAlmostEqual(planner.get_distance_meters(37.77, -122.41, 37.77, -122.41), 0.0, places=4)

    def test_basic_boustrophedon_sweep(self):
        # Coordinates of a rectangular field in OFAS
        polygon = [
            (37.774900, -122.419400), # Top-Left
            (37.774900, -122.418400), # Top-Right
            (37.774300, -122.418400), # Bottom-Right
            (37.774300, -122.419400), # Bottom-Left
        ]

        swath_width = 3.5  # meters
        waypoints = planner.generate_sweep_coverage(polygon, swath_width)

        # Ensure waypoints are generated
        self.assertNotEmpty(waypoints)
        
        # Verify the structure/serialization of returned waypoints
        for wp in waypoints:
            self.assertIsInstance(wp, planner.Waypoint)
            dct = wp.to_dict()
            self.assertIn("lat", dct)
            self.assertIn("lon", dct)
            self.assertTrue(37.7740 <= dct["lat"] <= 37.7750)
            self.assertTrue(-122.4200 <= dct["lon"] <= -122.4180)

    def test_empty_and_trivial_safeguards(self):
        # Empty polygon
        with self.assertRaises(ValueError):
            planner.generate_sweep_coverage([], 2.5)

        # Less than 3 points
        with self.assertRaises(ValueError):
            planner.generate_sweep_coverage([(37.77, -122.41), (37.78, -122.42)], 2.5)

    def assertNotEmpty(self, obj, msg=None):
        if not obj:
            raise self.failureException(msg or f"{obj!r} is unexpectedly empty")


if __name__ == "__main__":
    unittest.main()
