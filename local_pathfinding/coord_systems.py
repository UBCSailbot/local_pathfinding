"""Functions and classes for converting between coordinate systems."""

import math
from typing import NamedTuple

from pyproj import Geod

GEODESIC = Geod(ellps="WGS84")
M_TO_KM = 0.001


class LatLon(NamedTuple):
    """Geographical coordinate representation.

    Attributes:
        latitude (float): Latitude in degrees.
        longitude (float): Longitude in degrees.
    """

    latitude: float
    longitude: float


class XY(NamedTuple):
    """2D Cartesian coordinate representation.

    Attributes:
        x (float): X coordinate.
        y (float): Y coordinate.
    """

    x: float
    y: float


def cartesian_to_true_bearing(cartesian: float) -> float:
    """Convert a cartesian angle to the equivalent true bearing.

    Args:
        cartesian (float): Angle where 0 is east and values increase counter-clockwise.

    Returns:
        float: Angle where 0 is north and values increase clockwise.
    """
    return (90 - cartesian + 360) % 360


def latlon_to_xy(reference: LatLon, latlon: LatLon) -> XY:
    """Convert a geographical coordinate to a 2D Cartesian coordinate given a reference point.

    Args:
        reference (LatLon): Coordinate that will be the origin of the Cartesian coordinate system.
        latlon (LatLon): Coordinate to be converted to the Cartesian coordinate system.

    Returns:
        XY: The x and y components in km.
    """
    forward_azimuth_deg, _, distance_m = GEODESIC.inv(
        reference.longitude, reference.latitude, latlon.longitude, latlon.latitude
    )
    true_bearing = math.radians(forward_azimuth_deg)
    distance = distance_m * M_TO_KM

    return XY(
        x=distance * math.sin(true_bearing),
        y=distance * math.cos(true_bearing),
    )


def xy_to_latlon(reference: LatLon, xy: XY) -> LatLon:
    """Convert a 2D Cartesian coordinate to a geographical coordinate given a reference point.

    Args:
        reference (LatLon): Coordinate that is the origin of the Cartesian coordinate system.
        xy (XY): Coordinate to be converted to the geographical coordinate system.

    Returns:
        LatLon: The latitude and longitude in degrees.
    """
    true_bearing = math.degrees(math.atan2(xy.x, xy.y))
    distance = math.hypot(*xy) / M_TO_KM
    dest_lon, dest_lat, _ = GEODESIC.fwd(
        reference.longitude, reference.latitude, math.degrees(true_bearing), distance
    )

    return LatLon(latitude=dest_lat, longitude=dest_lon)
