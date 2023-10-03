import math

import numpy as np
from shapely.geometry import Point, Polygon


class Obstacle(Polygon):

    """
    This class describes general obstacle objects which are anything which the sailbot must avoid.
    Obstacles are a child class of Polygon.

    For now, it serves as a placeholder for future functionality, in case we need to collect all
    obstacles, of different types, in one place.

    """

    def __init__(self, position) -> None:
        """
        Obstacle constructor

        Parameters
        ----------
        position : x,y coordinates of the obstacle (likely converted from lat/lon)

        """
        self.position = position


class Boat(Obstacle):

    """
    This class describes boat objects.
    Boats may be stationary or moving. If moving, they have a speed and heading.


    The following information is obtained from AIS:
    - ID (MMSI)
    - position (lat/lon)
    - speed (knots)
    - heading (degrees measured clockwise from true north)




    """

    def __init__(self, id, position, speed, heading) -> None:
        """



        Parameters
        ----------
        arg1 : type
            Description of arg1.
        arg2 : type
            Description of arg2.

        Attributes
        ----------
        attribute1 : type
            Description of attribute1.
        attribute2 : type
            Description of attribute2.
        """
        super().__init__(position)
        self.id = id
        self.speed = speed
        self.heading = heading
