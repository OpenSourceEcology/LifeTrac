"""planner.py — Autonomy & Route Planning core for the LifeTrac v25 base station.

Provides helper routines for converting geofence polygons (in global GPS
coordinates) into sweeping, back-and-forth agricultural parallel rows
(boustrophedon path planning) adjusted for swath width, overlap, and
implement control.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class Waypoint:
    lat: float
    lon: float
    command: int = 0  # 0 = normal travel, 1 = lower implement, 2 = raise implement

    def to_dict(self) -> dict:
        return {
            "lat": self.lat,
            "lon": self.lon,
            "command": self.command
        }


def get_distance_meters(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Haversine formula to compute distance in meters between two GPS coordinates."""
    R = 6371000.0  # Earth's radius in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = (math.sin(delta_phi / 2.0) ** 2 +
         math.cos(phi1) * math.cos(phi2) * (math.sin(delta_lambda / 2.0) ** 2))
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c


def generate_sweep_coverage(geofence_polygon: List[Tuple[float, float]],
                             swath_width_m: float) -> List[Waypoint]:
    """Generates a back-and-forth (Boustrophedon) sweep pattern inside a geofence.

    Args:
        geofence_polygon: List of (lat, lon) vertices defining the field boundary.
        swath_width_m: Distance between parallel rows in meters.

    Returns:
        List of Waypoints representing the plan.
    """
    if len(geofence_polygon) < 3:
        raise ValueError("Geofence polygon must have at least 3 vertices to define a region.")

    # 1. Convert Geofence boundaries from lat/lon to a local flat scale (X, Y in meters)
    # Using the first vertex as the local origin (0, 0)
    origin_lat, origin_lon = geofence_polygon[0]
    local_coords = []

    for lat, lon in geofence_polygon:
        # Approximate delta-X and delta-Y meters from origin
        y = get_distance_meters(origin_lat, origin_lon, lat, origin_lon)
        if lat < origin_lat:
            y = -y
        x = get_distance_meters(origin_lat, origin_lon, origin_lat, lon)
        if lon < origin_lon:
            x = -x
        local_coords.append((x, y))

    # Find bounding box values
    xs = [pt[0] for pt in local_coords]
    ys = [pt[1] for pt in local_coords]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)

    # 2. Project horizontal/vertical scan-lines across the bounding box
    waypoints_local = []
    current_x = min_x + (swath_width_m / 2.0)
    direction_toggle = 1  # 1 = sweep up, -1 = sweep down

    while current_x < max_x:
        # Compute vertical row line segments in local space (Y coordinates range from min to max)
        if direction_toggle == 1:
            waypoints_local.append((current_x, min_y, 1))  # Start row (Command 1: Lower Implement)
            waypoints_local.append((current_x, max_y, 2))  # End row (Command 2: Raise Implement)
        else:
            waypoints_local.append((current_x, max_y, 1))  # Start row
            waypoints_local.append((current_x, min_y, 2))  # End row

        current_x += swath_width_m
        direction_toggle *= -1  # Alternate direction

    # 3. Convert generated local plan coordinates back to global Lat/Lon
    global_waypoints = []
    # 111,139 meters is approximately 1 degree of latitude.
    # Longitude depends on the cosine of the active latitude.
    m_per_deg_lat = 111139.0
    m_per_deg_lon = 111139.0 * math.cos(math.radians(origin_lat))

    for x, y, cmd in waypoints_local:
        lat = origin_lat + (y / m_per_deg_lat)
        lon = origin_lon + (x / m_per_deg_lon)
        global_waypoints.append(Waypoint(lat=lat, lon=lon, command=cmd))

    return global_waypoints
