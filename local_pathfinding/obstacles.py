"""Describes obstacles which the Sailbot must avoid: Boats and Land"""

import math

import numpy as np
from custom_interfaces.msg import HelperAISShip
from numpy.typing import NDArray
from shapely.geometry import Point, Polygon

from local_pathfinding.coord_systems import XY, LatLon, latlon_to_xy, meters_to_km

# Constants
PROJ_TIME_NO_COLLISION = 3  # hours
COLLISION_ZONE_SAFETY_BUFFER = 0.5  # km
COLLISION_CONE_STRETCH_FACTOR = 1.5  # This factor changes the scope/width of the collision cone


class Obstacle:
    """This class describes general obstacle objects which are
    anything which the sailbot must avoid.

    Attributes:
        reference (LatLon): Lat and lon position of the next global waypoint.
        sailbot_position (LatLon): Lat and lon position of SailBot.
        sailbot_speed (float): Speed of the SailBot in kmph.
        collision_zone (Polygon): Shapely Polygon object representing the
        obstacle's collision zone.
    """

    def __init__(self, reference: LatLon, sailbot_position: LatLon, sailbot_speed: float):
        self.reference = reference
        self.sailbot_position = latlon_to_xy(reference, sailbot_position)
        self.sailbot_speed = sailbot_speed

        # This is defined in child classes
        self.collision_zone = None

    def is_valid(self, reference: LatLon, point_latlon: LatLon) -> bool:
        """Checks if a point is contained the obstacle's collision zone.

        Args:
            reference (LatLon): Lat and lon position of the next global waypoint.
            point (Point): Shapely Point representing the state point to be checked.

        Returns:
            bool: True if the point is not within the obstacle's collision zone, false otherwise.
        """
        point = latlon_to_xy(reference, point_latlon)

        # contains() requires a shapely Point object as an argument
        point = Point(*point)

        if self.collision_zone is None:
            raise ValueError("Collision zone has not been initialized")

        return not self.collision_zone.contains(point)


class Boat(Obstacle):

    """Describes boat objects which Sailbot must avoid.
    Also referred to target ships or boat obstacles.

    Attributes:
        ais_ship (HelperAISShip): an AISShip object, containing information about the boat.

    """

    def __init__(
        self,
        reference: LatLon,
        sailbot_position: LatLon,
        sailbot_speed: float,
        ais_ship: HelperAISShip,
    ):
        super().__init__(reference, sailbot_position, sailbot_speed)

        self.id = ais_ship.id

        # Position of the boat in XY
        position = latlon_to_xy(
            self.reference, LatLon(ais_ship.lat_lon.latitude, ais_ship.lat_lon.longitude)
        )
        # for debugging purposes
        self.position = position

        self.collision_zone = self.create_collision_cone(
            ais_ship.dimensions.width,
            ais_ship.dimensions.length,
            position,
            ais_ship.sog.sog,
            ais_ship.cog.cog,
            self.sailbot_position,
            self.sailbot_speed,
        )

    def create_collision_cone(
        self,
        width: float,
        length: float,
        position: XY,
        speed_over_ground: float,
        course_over_ground: float,
        sailbot_position: XY,
        sailbot_speed: float,
    ) -> Polygon:
        """Creates a Shapely Polygon to represent the boat's collision zone,
        which is shaped like a cone.

        The polygon is oversized according to the collision zone safety buffer, for
        added assurrance that the boat will be entirely contained by the polygon.
        """

        # coordinates of the center of the boat
        x, y = position[0], position[1]

        width = meters_to_km(width)
        length = meters_to_km(length)

        # Calculate distance the boat will travel before soonest possible collision with Sailbot
        projected_distance = self.calculate_projected_distance(
            position,
            course_over_ground,
            speed_over_ground,
            sailbot_position,
            sailbot_speed,
        )

        # TODO This feels too arbitrary, maybe will incorporate ROT at a later time
        collision_cone_stretch = projected_distance * COLLISION_CONE_STRETCH_FACTOR

        # Points of the boat collision cone polygon before rotation and centred at the origin
        points = np.array(
            [
                [-width / 2, -length / 2],
                [-collision_cone_stretch * width, length / 2 + projected_distance],
                [collision_cone_stretch * width, length / 2 + projected_distance],
                [width / 2, -length / 2],
            ]
        )

        # Rotation matrix to rotate the polygon about the origin according to COG
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

        return Polygon(points).buffer(COLLISION_ZONE_SAFETY_BUFFER, join_style=2)

    def calculate_projected_distance(
        self,
        position: XY,
        course_over_ground: float,
        speed_over_ground_kmph: float,
        sailbot_position: XY,
        sailbot_speed: float,
    ) -> float:
        """Calculates the distance (km) the boat obstacle will travel before collision, if the
        Sailbot moves directly towards the soonest possible collision point at current speed.

        Args:
            position (XY): x,y coordinates of the boat in km.
            course_over_ground (float): COG of the boat in degrees clockwise from true north.
            speed_over_ground_kmph (float): Speed of the boat in km/h, over ground.
            sailbot_position (XY): x,y coordinates of the Sailbot in km.
            sailbot_speed (float): Speed of the Sailbot in km/h.
        """
        # Speed over ground vector of the boat obstacle
        boat_sog_vector = np.array(
            [
                speed_over_ground_kmph * np.sin(math.radians(course_over_ground)),
                speed_over_ground_kmph * np.cos(math.radians(course_over_ground)),
            ]
        )

        time_to_intersection = self.calculate_time_to_intersection(
            position, boat_sog_vector, sailbot_position, sailbot_speed
        )

        # Sailbot and this Boat will never collide
        if time_to_intersection < 0:
            return PROJ_TIME_NO_COLLISION * speed_over_ground_kmph

        return time_to_intersection * speed_over_ground_kmph

    def calculate_time_to_intersection(
        self,
        position: XY,
        boat_sog_vector: NDArray,
        sailbot_position: XY,
        sailbot_speed: float,
    ) -> float:
        """Calculates the time until the boat and Sailbot collide, if the Sailbot moves
        directly towards the soonest possible collision point at its current speed.
        The system is modeled by two parametric lines extending from the positions of the boat
        obstacle and sailbot respectively in 2D space. These lines may intersect at some specific
        point and time.

        This linear system, in which the vector that represents the Sailbot's velocity is free
        to point at the soonest possible collision point (but whose magnitude is known),
        is solved using linear algebra and  the quadratic formula.

        A more in-depth explanation for this function can be found here:
        https://ubcsailbot.atlassian.net/wiki/spaces/prjt22/pages/1881145358/Obstacle+Class+Planning

        Args:
            position (XY): x,y coordinates of the boat in km.
            boat_sog_vector (np.array): x,y components of the boat's speed over ground.
            sailbot_position (XY): x,y coordinates of the Sailbot in km.
            sailbot_speed (float): Speed of the Sailbot in kmph.

        Returns:
            t (float): Time in hours until the boat and Sailbot collide
            -1 if the boats will never collide.
        """
        v1 = boat_sog_vector[0]
        v2 = boat_sog_vector[1]
        a = position[0]
        b = position[1]
        c = sailbot_position[0]
        d = sailbot_position[1]

        quadratic_coefficients = np.array(
            [
                v1**2 + v2**2 - (sailbot_speed**2),
                2 * (v1 * (a - c) + v2 * (b - d)),
                (a - c) ** 2 + (b - d) ** 2,
            ]
        )

        # If the radicand of the quadratic formula is negative, there is no real time
        # when the boats will collide
        if (2 * (v1 * (a - c) - v2 * (b - d))) ** 2 - 4 * (
            v1**2 + v2**2 - (sailbot_speed**2)
        ) * (a - c) ** 2 + (b - d) ** 2 < 0:
            return -1

        # The solution to the quadratic formula is the time until the boats collide
        t = np.roots(quadratic_coefficients)

        # filter out only positive times
        t = [i for i in t if i > 0]

        if len(t) == 0:
            return -1
        else:
            # Return the smaller positive time, if there is one
            return min(t)
