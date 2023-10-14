import math

import numpy as np
from custom_interfaces.msg import HelperAISShip
from numpy.typing import NDArray
from shapely.geometry import Point, Polygon

from local_pathfinding.coord_systems import (
    XY,
    LatLon,
    knots_to_kilometers_per_hour,
    latlon_to_xy,
    meters_to_km,
)

# Constants
MAX_PROJECTION_TIME = 3  # hours


# TODO Look into adding more data available from AIS
class Obstacle:

    """
    This Interface describes general obstacle objects which are
    anything which the sailbot must avoid.

    The choice was made to create an interface rather than a class as Boat and LandMass objects
    may be very different objects which share an "avoidable" property and a position, but not much
    else.

    """

    def __init__(
        self,
        reference: LatLon,
        sailbot_position: LatLon,
        sailbot_speed: float,
    ):
        """
        Args:
            -reference (LatLon): latitude and longitude of the next global waypoint
            -sailbot_position (LatLon): lat and lon position of SailBot
            -sailbot_speed (float): speed of the SailBot in km/h
        """
        self.reference = reference

        # Position of the Sailbot in XY
        self.sailbot_position = latlon_to_xy(reference, sailbot_position)

        self.sailbot_speed = sailbot_speed

        # This should be defined in child classes
        self.collision_zone = Polygon(None, None)

    def is_valid(self, reference: LatLon, point_latlon: LatLon) -> bool:
        """
        Checks if a point is contained the obstacle's interior

        Args:
            point (Point): a shapely Point object representing the point to be checked

        Returns:
            bool: True if the point is not within the obstacle's interior, False otherwise
        """
        point = latlon_to_xy(reference, point_latlon)

        # contains() requires a shapely Point object as an argument
        point = Point(point[0], point[1])
        return not self.collision_zone.contains(point)


class Boat(Obstacle):

    """
    This class describes boat objects which Sailbot must avoid.
    Also referred to target ships or boat obstacles.


    Attributes
    ----------
        -id (int): MMSI number of the boat
        -speed_over_ground (float): speed of the boat, over ground, in knots
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
            - latitude (float) latitude in degrees
            - longitude (float) longitude in degrees
            - speed over ground (float) speed in knots over ground
            - course over ground (float) degrees measured clockwise from true north
            - boat dimensions (float) width and length of the boat in meters
            - rate of turn (float) ROT in AISROT scale -126 to +126 corresponding to
                -708 to +708 deg/min

            - other information is also available but not used here

        To avoid confusion, the physical dimensions, position, and COG of the boat are
        stored in the aggregated Polygon object, not the Boat object itself.
        So that all we need to do to update the boat's position or COG is to update the
        polygon's position, size, and rotation.

        points (x,y) are measured in km, with the origin set at the current global waypoint.

    """

    def __init__(
        self,
        reference: LatLon,
        sailbot_position: LatLon,
        sailbot_speed: float,
        ais_ship: HelperAISShip,
    ):
        """
        Args:
            -reference (LatLon): latitude and longitude of the next global waypoint
            -sailbot_position (LatLon): lat and lon position of SailBot
            -sailbot_speed (float): speed of the SailBot in km/h
            -ais_ship (HelperAISShip): an AISShip object, containing the following information:
                -id (int): MMSI number of the boat
                -width (float): width of the boat in meters
                -length (float): length of the boat in meters
                -lat_lon (LatLon): latitude and longitude of the boat
                -speed_over_ground (float): speed of the boat in knots, over ground
                -course_over_ground (float): COG of the boat, in degrees, clockwise from true north
                -rate_of_turn (float): ROT of the boat in AISROT scale -126 to +126 corresponding
                    to -708 to +708 degrees per minute
        """
        super().__init__(reference, sailbot_position, sailbot_speed)

        self.id = ais_ship.id

        # Position of the boat in XY
        position = latlon_to_xy(
            self.reference, LatLon(ais_ship.lat_lon.latitude, ais_ship.lat_lon.longitude)
        )
        # A Boat's collision zone is represented by a cone shaped polygon
        self.collision_zone = self.create_collision_cone(
            ais_ship.dimensions.width,
            ais_ship.dimensions.length,
            position,
            ais_ship.sog.sog,
            ais_ship.cog.cog,
            ais_ship.rot.rot,
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
        rate_of_turn: float,
        sailbot_position: XY,
        sailbot_speed: float,
    ) -> Polygon:
        """
        Creates a Shapely Polygon to represent the boat's collision_cone, which is sized,
        orientated and positioned according to the boat's COG, SOG, and position.

        Args:
            width (float): width of the boat in meters
            length (float): length of the boat in meters
            position (XY): x,y coordinates of the boat in km
            speed_over_ground (float): speed of the boat in knots, over ground
            course_over_ground (float): COG of the boat in degrees clockwise from true north
            rate_of_turn (float): ROT of the boat in AISROT scale -126 to +126 corresponding to
                -708 to +708 degrees per minute
            Sailbot_position (XY): x,y coordinates of the Sailbot in km

        Notes:
            ROT is not incorporated yet, but may be in the future.
        """

        # coordinates of the center of the boat
        x, y = position[0], position[1]

        speed_over_ground_kmph = knots_to_kilometers_per_hour(speed_over_ground)

        width = meters_to_km(width)
        length = meters_to_km(length)

        # Calculate distance the boat will travel before soonest possible collision with Sailbot
        projected_distance = self.calculate_projected_distance(
            position,
            course_over_ground,
            speed_over_ground_kmph,
            sailbot_position,
            sailbot_speed,
        )

        # This factor can be adjusted to change the scope/width of the collision cone
        # TODO This feels too arbitrary, maybe will incorporate ROT at a later time
        COLLISION_CONE_STRETCH_FACTOR = projected_distance * 1.5

        # Points of the boat collision box polygon before rotation and centred at the origin
        points = np.array(
            [
                [-width / 2, -length / 2],
                [-COLLISION_CONE_STRETCH_FACTOR * width, length / 2 + projected_distance],
                [COLLISION_CONE_STRETCH_FACTOR * width, length / 2 + projected_distance],
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

        return Polygon(points)

    def calculate_projected_distance(
        self,
        position: XY,
        course_over_ground: float,
        speed_over_ground_kmph: float,
        sailbot_position: XY,
        sailbot_speed: float,
    ) -> float:
        """
        Calculates the distance (km) the boat obstacle will travel before collision, if the Sailbot
        moves directly towards the soonest possible collision point at current speed.

        Args:
            position (XY): x,y coordinates of the boat in km
            course_over_ground (float): COG of the boat in degrees clockwise from true north
            speed_over_ground_kmph (float): speed of the boat in km/h, over ground
            sailbot_position (XY): x,y coordinates of the Sailbot in km
            sailbot_speed (float): speed of the Sailbot in km/h
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

        if time_to_intersection < 0:
            return MAX_PROJECTION_TIME * speed_over_ground_kmph

        return time_to_intersection * speed_over_ground_kmph

    def calculate_time_to_intersection(
        self,
        position: XY,
        boat_sog_vector: NDArray,
        sailbot_position: XY,
        sailbot_speed: float,
    ) -> float:
        """
        Calculates the time until the boat and Sailbot collide, if the Sailbot moves
        directly towards the soonest possible collision point at its current speed.
        The system is modeled by two parametric lines extending from the positions of the boat
        obstacle and sailbot respectively in 2D space. These lines may intersect at some specific
        point and time.

        This linear system, in which the vector that represents the Sailbot's velocity is free
        to point at the soonest possible collision point (but whose magnitude is known),
        is solved using linear algebra and  the quadratic formula.

        Args:
            position (XY): x,y coordinates of the boat in km
            boat_sog_vector (np.array): x,y components of the boat's speed over ground
            sailbot_position (XY): x,y coordinates of the Sailbot in km
        Returns:
            time_to_intersection (float): time in hours until the boat and Sailbot collide
            -1 if the boats will never collide
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
                2 * (v1 * (a - c) - v2 * (b - d)),
                (a - c) ** 2 + (b - d) ** 2,
            ]
        )

        # If the radicand  of the quadratic formula is negative, the boats will never collide
        if (2 * (v1 * (a - c) - v2 * (b - d))) ** 2 - 4 * (
            v1**2 + v2**2 - (sailbot_speed**2)
        ) * (a - c) ** 2 + (b - d) ** 2 <= 0:
            return -1

        # The solution to the quadratic formula is the time until the boats collide
        t = np.roots(quadratic_coefficients)

        # Return the smaller positive time
        return min([i for i in t if i > 0])
