"""Describes obstacles which the Sailbot must avoid: Boats and Land"""

import math

import numpy as np
from custom_interfaces.msg import HelperAISShip
from shapely import affinity as af
from shapely.geometry import Point, Polygon

from local_pathfinding.coord_systems import XY, LatLon, latlon_to_xy, meters_to_km

# Constants
PROJ_TIME_NO_COLLISION = 3  # hours
COLLISION_ZONE_SAFETY_BUFFER = 0.5  # km
COLLISION_ZONE_STRETCH_FACTOR = 1.5  # This factor changes the scope/width of the collision cone


class Obstacle:
    """This class describes general obstacle objects which are
    anything which the sailbot must avoid.

    Attributes:
        reference (LatLon): Lat and lon position of the next global waypoint.
        sailbot_position (XY): Lat and lon position of SailBot.
        sailbot_speed (float): Speed of the SailBot in kmph.
        collision_zone (Optional[Polygon]): Shapely polygon representing the
            obstacle's collision zone. Shape depends on the child class.
    """

    def __init__(self, reference: LatLon, sailbot_position: LatLon, sailbot_speed: float):
        self.reference = reference
        self.sailbot_position_latlon = sailbot_position
        self.sailbot_position = latlon_to_xy(self.reference, self.sailbot_position_latlon)
        self.sailbot_speed = sailbot_speed

        # Defined later by the child class
        self.collision_zone = None

    def is_valid(self, point: XY) -> bool:
        """Checks if a point is contained the obstacle's collision zone.

        Args:
            point (LatLon): LatLon Point representing the state point to be checked.

        Returns:
            bool: True if the point is not within the obstacle's collision zone, false otherwise.

        Raises:
            ValueError: If the collision zone has not been initialized.
        """
        if self.collision_zone is None:
            raise ValueError("Collision zone has not been initialized")

        # contains() requires a shapely Point object as an argument
        point = Point(*point)

        return not self.collision_zone.contains(point)

    def update_collision_zone(self, collision_zone: Polygon, centre_position: XY):
        """Updates the collision zone of the obstacle. Called by the child classes.

        Args:
            collision_zone (Polygon): Shapely Polygon representing the obstacle's collision zone.
        """
        collision_zone = af.translate(collision_zone, *centre_position)
        self.collision_zone = collision_zone.buffer(COLLISION_ZONE_SAFETY_BUFFER, join_style=2)

    def update_sailbot_data(self, sailbot_position: LatLon, sailbot_speed: float):
        """Updates the sailbot's position and speed.

        Args:
            sailbot_position (LatLon): LatLon position of the SailBot.
            sailbot_speed (float): Speed of the SailBot in kmph.
        """
        self.sailbot_position_latlon = sailbot_position
        self.sailbot_position = latlon_to_xy(self.reference, sailbot_position)
        self.sailbot_speed = sailbot_speed

    def update_reference_point(self, reference: LatLon):
        """Updates the reference point.

        Args:
            reference (LatLon): LatLon position of the updated global waypoint.
        """
        self.reference = reference
        self.sailbot_position = latlon_to_xy(self.reference, self.sailbot_position_latlon)

        if isinstance(self, Boat):
            # regenerate collision zone with updated reference point
            self.update_boat_collision_zone()


class Boat(Obstacle):
    """Describes boat objects which Sailbot must avoid.
    Also referred to target ships or boat obstacles.

    Attributes:
       ais_ship (HelperAISShip): AIS Ship message containing information about the boat.
    """

    def __init__(
        self,
        reference: LatLon,
        sailbot_position: LatLon,
        sailbot_speed: float,
        ais_ship: HelperAISShip,
    ):
        super().__init__(reference, sailbot_position, sailbot_speed)

        self.ais_ship = ais_ship
        self.update_boat_collision_zone()

    def update_boat_collision_zone(self, ais_ship: HelperAISShip = None):
        """Sets or regenerates a Shapely Polygon that represents the boat's collision zone,
        which is shaped like a cone.

        Args:
            ais_ship (Optional [HelperAISShip]): AIS Ship message containing boat information.

        Returns:
            Polygon: Shapely Polygon representing the boat's collision zone.
        """
        if ais_ship is not None:
            if ais_ship.id != self.ais_ship.id:
                raise ValueError("Argument AIS Ship ID does not match this Boat instance's ID")

            # Ensure ais_ship instance variable is the most up to date one
            self.ais_ship = ais_ship

        # coordinates of the center of the boat
        position = latlon_to_xy(
            self.reference, LatLon(self.ais_ship.lat_lon.latitude, self.ais_ship.lat_lon.longitude)
        )

        width = meters_to_km(self.ais_ship.width.dimension)
        length = meters_to_km(self.ais_ship.length.dimension)

        # Course over ground of the boat
        cog = self.ais_ship.cog.heading

        # Calculate distance the boat will travel before soonest possible collision with Sailbot
        projected_distance = Boat.calculate_projected_distance(
            self.reference, self.sailbot_position, self.sailbot_speed, self.ais_ship
        )

        # TODO This feels too arbitrary, maybe will incorporate ROT at a later time
        collision_zone_stretch = projected_distance * COLLISION_ZONE_STRETCH_FACTOR

        # Points of the boat collision cone polygon before rotation and centred at the origin
        boat_collision_zone = Polygon(
            [
                [-width / 2, -length / 2],
                [-collision_zone_stretch * width, length / 2 + projected_distance],
                [collision_zone_stretch * width, length / 2 + projected_distance],
                [width / 2, -length / 2],
            ]
        )

        boat_collision_zone = af.rotate(boat_collision_zone, -cog, origin=(0, 0))

        self.update_collision_zone(boat_collision_zone, position)

    @staticmethod
    def calculate_projected_distance(
        reference: LatLon,
        sailbot_position: XY,
        sailbot_speed: float,
        ais_ship: HelperAISShip,
    ) -> float:
        """Calculates the distance the boat obstacle will travel before collision, if
        Sailbot moves directly towards the soonest possible collision point at its current speed.
        The system is modeled by two parametric lines extending from the positions of the boat
        obstacle and sailbot respectively, in 2D space. These lines may intersect at some specific
        point and time.

        The vector that represents the Sailbot's velocity is free to point at the soonest possible
        collision point, but its magnitude is constrained.

        An in-depth explanation for this function can be found here:
        https://ubcsailbot.atlassian.net/wiki/spaces/prjt22/pages/1881145358/Obstacle+Class+Planning


        Returns:
            float: Distance the boat will travel before collision or the max projection distance
                   if a collision is not possible.

        """
        sog = ais_ship.sog.speed
        cog = ais_ship.cog.heading
        position = latlon_to_xy(
            reference, LatLon(ais_ship.lat_lon.latitude, ais_ship.lat_lon.longitude)
        )

        # vector components of the boat's speed over ground
        v1 = sog * np.sin(math.radians(cog))
        v2 = sog * np.cos(math.radians(cog))

        # coordinates of the boat
        a = position[0]
        b = position[1]

        # coordinates of Sailbot
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
            print("no real times")
            # Sailbot and this Boat will never collide
            return PROJ_TIME_NO_COLLISION * ais_ship.sog.speed

        # The solution to the quadratic formula is the time until the boats collide
        roots = np.roots(quadratic_coefficients)

        # filter out only positive roots
        roots = [i for i in roots if i >= 0]

        if len(roots) == 0:
            print("no positive times")
            # Sailbot and this Boat will never collide
            return PROJ_TIME_NO_COLLISION * ais_ship.sog.speed

        else:
            # Use the smaller positive time, if there is one
            t = min(roots)

        return t * ais_ship.sog.speed
