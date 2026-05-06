"""Geographic helpers used by the visualization tool.

Converts (lat, lon, alt) to local ENU (east, north, up) metres around a
fixed origin. The approximation is the standard equirectangular one,
good to <1% over a few hundred metres at sub-polar latitudes - more
than enough for VTOL flight visualization.
"""

from __future__ import annotations

import math
from typing import Tuple

EARTH_RADIUS_M = 6378137.0


def lla_to_enu(lat_deg: float, lon_deg: float, alt_m: float,
               origin_lat_deg: float, origin_lon_deg: float,
               origin_alt_m: float) -> Tuple[float, float, float]:
    """Equirectangular lat/lon/alt -> local ENU metres around origin."""
    lat0 = math.radians(origin_lat_deg)
    d_lat = math.radians(lat_deg - origin_lat_deg)
    d_lon = math.radians(lon_deg - origin_lon_deg)
    east = EARTH_RADIUS_M * d_lon * math.cos(lat0)
    north = EARTH_RADIUS_M * d_lat
    up = float(alt_m) - float(origin_alt_m)
    return east, north, up


def lla_array_to_enu(lats, lons, alts, origin_lat, origin_lon, origin_alt):
    """Vectorized version. Accepts numpy arrays, returns (E, N, U) arrays."""
    import numpy as np
    lats = np.asarray(lats, dtype=float)
    lons = np.asarray(lons, dtype=float)
    alts = np.asarray(alts, dtype=float)
    lat0 = math.radians(float(origin_lat))
    d_lat = np.radians(lats - float(origin_lat))
    d_lon = np.radians(lons - float(origin_lon))
    east = EARTH_RADIUS_M * d_lon * math.cos(lat0)
    north = EARTH_RADIUS_M * d_lat
    up = alts - float(origin_alt)
    return east, north, up
