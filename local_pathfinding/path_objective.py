import math
import sys

from ompl import util as ou
from ompl import base as ob


class Distanceobjective():

    def __init__(self, space_information):
        self.ss = space_information

    def get_path_length_objective(self):
        path_objective = ob.PathLengthOptimizationObjective(self.ss)

        return path_objective

    def get_euclidean_path_length_objective(self, s1, s2):
        distance = (s1^2 + s2^2)^0.5

        print(dir(self.ss))
        print(self.ss.distance)

        return None
