import math
import sys

from ompl import util as ou
from ompl import base as ob

class Distanceobjective():

    def __init__(self, simple_setup, heading_degrees):
        self.si_ = simple_setup
        self.last_direction_radians = math.radians(heading_degrees)

    def get_path_length_objective(si):
        return ob.PathLengthOptimizationObjective(si)

    def segmenttimelength(self):
        pass

    def averagespeed(self):
        pass


def getSailingObjective(simple_setup, windDirectionDegrees, headingDegrees):
    lengthObj = ob.PathLengthOptimizationObjective(simple_setup)

    opt = ob.MultiOptimizationObjective(simple_setup)

    opt.addObjective(lengthObj, 1000)

    return opt
