import math

import numpy as np
from shapely.geometry import Polygon


# TODO Look into adding more data available from AIS
# TODO ensure proper use of true_bearing and heading
# TODO change the boat's polygon to represent its possible positions in a given time frame
class ObstacleInterface:

    """
    This Interface describes general obstacle objects which are
    anything which the sailbot must avoid.

    The choice was made to create an interface rather than a class as Boat and LandMass objects
    may be very different objects which share an "avoidable" property, but not much else.

    For now, it serves as a placeholder for future functionality, in case we need to collect all
    obstacles, of different types, in one place.

    """


class Boat(ObstacleInterface):

    """
    This class describes boat objects.

    Attributes
    ----------
        -id (int): MMSI number of the boat
        -speed (float): speed of the boat in knots
        -shell (Polygon): a shapely Polygon object representing the boat's hull
            -the shell is orientated according to the heading of the boat
            -position (XY): x,y coordinates of the boat's center (converted from lat/lon)
            -true bearing (float): heading of the boat in degrees, clockwise from true north
            -width (float): width of the boat in meters
            -length (float): length of the boat in meters
    Notes
    ----------
        The following information about a boat is obtained from AIS:
            - Maritime Mobile Service Identity (9 digit int)
            - latitude (float)
            - longitude (float)
            - speed (knots)
            - heading (degrees measured clockwise from true north)
            - other information is also available but not used here
            - In the future, I will want to obtain ship size and rate of turn

        To avoid confusion, the physical dimensions, position, and heading of the boat are
        stored in the aggregated Polygon object, not the Boat object itself.
        So that all we need to do to update the boat's position or true bearing is to update the
        polygon's position and rotation.

    """

    def __init__(self, id, width, length, position, speed, true_bearing):
        """
        Args:

            -id (int): MMSI number of the boat
            -width (float): width of the boat in meters
            -length (float): length of the boat in meters
            -position (XY): x,y coordinates of the boat converted from lat/lon
            -speed (float): speed of the boat in knots
            -true_bearing (float, degrees): heading of the boatclockwise from true north
        """
        self.id = id
        self.speed = speed
        self.hull = self.create_hull(position, width, length, true_bearing)

    def create_hull(self, position, width, length, true_bearing) -> Polygon:
        """
        creates a Shapely Polygon to represent the boat's hull, which is orientated and positioned
        according to the boat's true bearing and position.

        Args:

            position (XY): x,y coordinates of the boat, converted from lat/lon
            width (float): width of the boat in meters
            length (float): length of the boat in meters
            true_bearing (float): heading of the boat in degrees clockwise from true north
        """
        # coordinates of the center of the boat
        x, y = position[0], position[1]

        # Points of the boat polygon before rotation and centred at the origin
        points = np.array(
            [
                [-width / 2, -length / 2],
                [-width / 2, length / 2],
                [width / 2, length / 2],
                [width / 2, -length / 2],
            ]
        )

        # Rotation matrix
        # according to napkin math, the rotation matrix should be able to
        # use the true_bearing angle directly
        # but #TODO: check this
        rot = np.array(
            [
                [np.cos(math.radians(true_bearing)), np.sin(math.radians(true_bearing))],
                [-np.sin(math.radians(true_bearing)), np.cos(math.radians(true_bearing))],
            ]
        )

        # rotate the points about the origin, to orientate the boat according to its heading
        points = points @ rot

        # translate the points to the boat's position
        points = points + np.array([x, y])

        return Polygon(points)
