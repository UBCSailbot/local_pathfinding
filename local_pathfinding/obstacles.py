"""Describes obstacles which the Sailbot must avoid: Boats and Land"""

import math

import numpy as np
from custom_interfaces.msg import HelperAISShip
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
        sailbot_position (XY): Lat and lon position of SailBot.
        sailbot_speed (float): Speed of the SailBot in kmph.
        collision_zone (Polygon): Shapely Polygon representing the obstacle's collision zone.
    """

    def __init__(self, reference: LatLon, sailbot_position: LatLon, sailbot_speed: float):
        self.reference = reference
        self.sailbot_position = latlon_to_xy(reference, sailbot_position)
        self.sailbot_speed = sailbot_speed

        # This is defined in child classes
        self.collision_zone = None

    def is_valid(self, point: XY) -> bool:
        """Checks if a point is contained the obstacle's collision zone.

        Args:
            point (LatLon): LatLon Point representing the state point to be checked.

        Returns:
            bool: True if the point is not within the obstacle's collision zone, false otherwise.
        """

        # contains() requires a shapely Point as an argument
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

        # Position of the boat should be converted from LatLon to XY
        position = latlon_to_xy(
            self.reference, LatLon(ais_ship.lat_lon.latitude, ais_ship.lat_lon.longitude)
        )

        self.id = ais_ship.id
        self.width = ais_ship.width.dimension
        self.length = ais_ship.length.dimension
        self.position = position
        self.sog = ais_ship.sog.speed
        self.cog = ais_ship.cog.heading
        self.collision_zone = self.create_collision_cone()

    def create_collision_cone(self) -> Polygon:
        """Creates a Shapely Polygon to represent the boat's collision zone,
        which is shaped like a cone.

        The polygon is oversized according to the collision zone safety buffer, for
        added assurrance that the boat will be entirely contained by the polygon.
        """

        # coordinates of the center of the boat
        x, y = self.position[0], self.position[1]

        width = meters_to_km(self.width)
        length = meters_to_km(self.length)

        # Calculate distance the boat will travel before soonest possible collision with Sailbot
        projected_distance = self.calculate_projected_distance()

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
        # self.cog is negative to reflect a CW rotation
        rot = np.array(
            [
                [
                    np.cos(math.radians(-self.cog)),
                    np.sin(math.radians(-self.cog)),
                ],
                [
                    -np.sin(math.radians(-self.cog)),
                    np.cos(math.radians(-self.cog)),
                ],
            ]
        )

        # rotate the points about the origin, to orientate the boat according to its COG
        points = points @ rot

        # translate the points to the boat's position
        points = points + np.array([x, y])

        return Polygon(points).buffer(COLLISION_ZONE_SAFETY_BUFFER, join_style=2)

    def calculate_projected_distance(self) -> float:
        """Calculates the distance (km) the boat obstacle will travel before collision, if the
        Sailbot moves directly towards the soonest possible collision point at current speed.

        Returns:
            float: Distance in km the boat will travel before collision
        """

        time_to_intersection = self.calculate_time_to_intersection()

        if time_to_intersection < 0:
            # Sailbot and this Boat will never collide
            return PROJ_TIME_NO_COLLISION * self.sog

        return time_to_intersection * self.sog

    def calculate_time_to_intersection(self) -> float:
        """Calculates the time until the boat and Sailbot collide, if the Sailbot moves
        directly towards the soonest possible collision point at its current speed.
        The system is modeled by two parametric lines extending from the positions of the boat
        obstacle and sailbot respectively, in 2D space. These lines may intersect at some specific
        point and time.

        This linear system, in which the vector that represents the Sailbot's velocity is free
        to point at the soonest possible collision point (but whose magnitude is constrained),
        is solved using linear algebra and the quadratic formula.

        A more in-depth explanation for this function can be found here:
        https://ubcsailbot.atlassian.net/wiki/spaces/prjt22/pages/1881145358/Obstacle+Class+Planning

        Returns:
            float: Time in hours until the boat and Sailbot collide
            -1 if the boats will never collide.
        """
        # vector components of the boat's speed over ground
        v1 = self.sog * np.sin(math.radians(self.cog))
        v2 = self.sog * np.cos(math.radians(self.cog))

        # coordinates of the boat
        a = self.position[0]
        b = self.position[1]

        # coordinates of Sailbot
        c = self.sailbot_position[0]
        d = self.sailbot_position[1]

        quadratic_coefficients = np.array(
            [
                v1**2 + v2**2 - (self.sailbot_speed**2),
                2 * (v1 * (a - c) + v2 * (b - d)),
                (a - c) ** 2 + (b - d) ** 2,
            ]
        )

        # If the radicand of the quadratic formula is negative, there is no real time
        # when the boats will collide
        if (2 * (v1 * (a - c) - v2 * (b - d))) ** 2 - 4 * (
            v1**2 + v2**2 - (self.sailbot_speed**2)
        ) * (a - c) ** 2 + (b - d) ** 2 < 0:
            return -1

        # The solution to the quadratic formula is the time until the boats collide
        t = np.roots(quadratic_coefficients)

        # filter out only positive roots
        t = [i for i in t if i > 0]

        if len(t) == 0:
            return -1
        else:
            # Return the smaller positive time, if there is one
            return min(t)
