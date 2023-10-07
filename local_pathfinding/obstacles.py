import math

import numpy as np
from shapely.geometry import Polygon


# TODO Look into adding more data available from AIS
# TODO implement isValid (need to decide on shape of polygon and how to calculate it)
# TODO change the boat's polygon to represent its possible positions in a given time frame
class ObstacleInterface:

    """
    This Interface describes general obstacle objects which are
    anything which the sailbot must avoid.

    The choice was made to create an interface rather than a class as Boat and LandMass objects
    may be very different objects which share an "avoidable" property and a position, but not much
    else.

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
        -collision_cone (Polygon): a shapely Polygon object representing the boat's collision box
            -the collision_cone represents the boats hull via a region of possible positions
            -It is shaped like a cone, with the tip at the boat's position and the base at the
             furthest possible position of the boat in the next time step
            -the collision_cone is orientated according to the course of the boat
            -position (XY): x,y coordinates of the boat's center (converted from lat/lon)
            -course_over_ground (float): course of the boat in degrees, clockwise from true north
            (#TODO: check if we need to consider magnetic north)
            -width (float): width of the boat in meters
            -length (float): length of the boat in meters
    Notes
    ----------
        The following information about a boat is obtained from AIS:
            - Maritime Mobile Service Identity (9 digit int)
            - latitude (float)
            - longitude (float)
            - speed (knots)
            - Course over ground (COG) (degrees measured clockwise from true north)
            - other information is also available but not used here
            - In the future, I may want to obtain ship size and rate of turn

        To avoid confusion, the physical dimensions, position, and COG of the boat are
        stored in the aggregated Polygon object, not the Boat object itself.
        So that all we need to do to update the boat's position or COG is to update the
        polygon's position, size, and rotation.

        points (x,y) are measured in meters, with the origin set at the current waypoint.

    """

    def __init__(self, id, width, length, position, speed, delta_time, course_over_ground):
        """
        Args:

            -id (int): MMSI number of the boat
            -width (float): width of the boat in meters
            -length (float): length of the boat in meters
            -position (XY): x,y coordinates of the boat converted from lat/lon
            -speed (float): speed of the boat in knots
            -delta_time (float): time in seconds, to calculate the boat's expected position
            -course_over_ground (float, degrees): COG of the boat, clockwise from true north
        """
        self.id = id
        self.speed = speed

        self.collision_cone = self.create_collision_cone(
            width, length, position, speed, delta_time, course_over_ground
        )

    def create_collision_cone(
        self, width, length, position, speed, delta_time, course_over_ground
    ) -> Polygon:
        """
        Creates a Shapely Polygon to represent the boat's collision_cone, which is sized,
        orientated and positioned according to the boat's COG, speed, and position.

        Args:
            width (float): width of the boat in meters
            length (float): length of the boat in meters
            position (XY): x,y coordinates of the boat, converted from lat/lon
            speed (float): speed of the boat in knots
            delta_time (float): time in seconds, to calculate the boat's expected position
            course_over_ground (float): COG of the boat in degrees clockwise from true north
        """

        def knots_to_meters_per_second(knots) -> float:
            return knots * 0.514444

        # This factor can be adjusted to change the scope of the collision cone
        COLLISION_CONE_STRETCH_FACTOR = 0.75

        speed_mps = knots_to_meters_per_second(speed)

        # These two lines use geometry of similar triangles to calculate the length and width of
        # the collision cone
        # See external documentation for a diagram
        # collision_cone_length = speed_mps * delta_time + COLLISION_CONE_STRETCH_FACTOR * length

        collision_cone_width = (
            (1 / 2)
            * math.sqrt(width**2 + length**2)
            * (1 + (speed_mps * delta_time) / COLLISION_CONE_STRETCH_FACTOR * length)
        )

        # coordinates of the center of the boat
        x, y = position[0], position[1]

        # Points of the boat polygon before rotation and centred at the origin
        points = np.array(
            [
                [0, -COLLISION_CONE_STRETCH_FACTOR * length],
                [collision_cone_width / 2, speed_mps * delta_time],
                [-collision_cone_width / 2, speed_mps * delta_time],
            ]
        )

        # Rotation matrix
        # according to napkin math, the rotation matrix should be able to
        # use the course_over_ground angle directly
        # but #TODO: check this
        rot = np.array(
            [
                [
                    np.cos(math.radians(-course_over_ground)),
                    np.sin(math.radians(-course_over_ground)),
                ],
                [
                    -np.sin(math.radians(-course_over_ground)),
                    np.cos(math.radians(-course_over_ground)),
                ],
            ]
        )

        # rotate the points about the origin, to orientate the boat according to its COG
        points = points @ rot

        # translate the points to the boat's position
        points = points + np.array([x, y])

        return Polygon(points)
