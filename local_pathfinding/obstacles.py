import numpy as np
from shapely.geometry import Point, Polygon


class Obstacle(Polygon):

    """
    This class describes general obstacle objects which are anything which the sailbot must avoid.
    Obstacles are a child class of Polygon.

    For now, it serves as a placeholder for future functionality, in case we need to collect all
    obstacles, of different types, in one place.

    """

    def __init__(self, position):
        """
        Obstacle constructor

        Args
        ----------
        position (NamedTuple) : x,y coordinates of the obstacle converted from lat/lon

        """
        self.position = position


class Boat(Obstacle):

    """
    This class describes boat objects.

    Attributes
    ----------
        -id (int): MMSI number of the boat
        -position (NamedTuple): x,y coordinates of the boat converted from lat/lon
        -speed (float): speed of the boat in knots
        -true bearing (float): heading of the boat in degrees measured clockwise from true north
        -width (float): width of the boat in meters
        -length (float): length of the boat in meters
        -shell (Polygon): a shapely Polygon object representing the boat's hull
            -the shell is orientated according to the heading of the boat

    Notes
    ----------
        The following information about a boat is obtained from AIS:
            - Maritime Mobile Service Identity (9 digit int)
            - latitude (float)
            - longitude (float)
            - speed (knots)
            - heading (degrees measured clockwise from true north)
            - other information is also available but not used here

    """

    def __init__(self, id, position, speed, true_bearing, width, length):
        """
        Args:

            id (int): MMSI number of the boat
            -position (NamedTuple): x,y coordinates of the boat converted from lat/lon
            -speed (float): speed of the boat in knots
            -true_bearing (float, degrees): heading of the boat measured clockwise from true north
            -width (float): width of the boat in meters
            -length (float): length of the boat in meters
        """
        super().__init__(position)
        self.id = id
        self.speed = speed
        self.true_bearing = true_bearing
        self.width = width
        self.length = length
        self.shell = self.create_hull()

    def create_hull(self, position, width, length, heading):
        """
        creates a Shapely Shell to represent the boat's hull, which is orientated and positioned
        according to the boat's true bearing and position.

        Args:

            position (NamedTuple): x,y coordinates of the boat, converted from lat/lon
            width (float): width of the boat in meters
            length (float): length of the boat in meters
            heading (float): heading of the boat in degrees measured clockwise from true north
        """
        # TODO implement this
