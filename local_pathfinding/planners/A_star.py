#!/usr/bin/env python

import heapq
import math

from ompl import base as ob
from ompl import geometric as og


class Node:
    """A node class for A* pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return (
            self.position.getX() == other.position.getX()
            and self.position.getY() == other.position.getY()
        )
        # return self.position[0] == other.position[0] and self.position[1] == other.position[1]

    def __str__(self):
        return (
            str(self.position.getX())
            + ", "
            + str(self.position.getY())
            + ", "
            + str(self.position.getYaw() * 180 / math.pi)
        )

    def __lt__(self, other):
        return self.f < other.f

    def __gt__(self, other):
        return self.f > other.f


class Astar(ob.Planner):
    def __init__(self, si: ob.SpaceInformation):
        super(Astar, self).__init__(si, "Astar")
        self.states_ = []
        self.sampler_ = si.allocStateSampler()

    def solve(self, ptc: ob.PlannerTerminationCondition):
        pdef = self.getProblemDefinition()  # type: ob.ProblemDefinition
        goal = pdef.getGoal()  # type: ob.GoalState
        si = self.getSpaceInformation()  # type: ob.SpaceInformation
        pi = self.getPlannerInputStates()  # type: ob.PlannerInputStates
        st = pi.nextStart()  # type: ob.State
        while st:
            self.states_.append(st)
            st = pi.nextStart()
        solution = None
        approxsol = 0
        # approxdif = 1e6
        start_state = pdef.getStartState(0)
        goal_state = goal.getState()
        start_node = Node(None, start_state)
        start_node.g = start_state.h = start_node.f = 0
        end_node = Node(None, goal_state)
        end_node.g = end_node.h = end_node.f = 0

        open_list = []
        closed_list = []
        heapq.heapify(open_list)
        adjacent_squares = (
            (1, 0, 0),
            (1, 1, 45),
            (0, 1, 90),
            (-1, 1, 135),
            (-1, 0, 0),
            (-1, -1, -135),
            (0, -1, -90),
            (1, -1, -45),
        )

        heapq.heappush(open_list, start_node)
        while len(open_list) > 0 and not ptc():
            current_node = heapq.heappop(open_list)
            if current_node == end_node:  # if we hit the goal
                current = current_node
                path = []
                while current is not None:
                    path.append(current.position)
                    current = current.parent
                for i in range(1, len(path)):
                    self.states_.append(path[len(path) - i - 1])
                solution = len(self.states_)
                break
            closed_list.append(current_node)

            children = []
            for new_position in adjacent_squares:
                node_position = si.allocState()
                current_node_x = current_node.position.getX()
                current_node_y = current_node.position.getY()
                node_position.setXY(
                    current_node_x + new_position[0], current_node_y + new_position[1]
                )
                node_position.setYaw(new_position[2] * math.pi / 180)

                if not si.checkMotion(current_node.position, node_position):
                    continue
                if not si.satisfiesBounds(node_position):
                    continue
                new_node = Node(current_node, node_position)
                children.append(new_node)

            for child in children:
                if child in closed_list:
                    continue
                if child.position.getYaw() % (math.pi / 2) == 0:
                    child.g = current_node.g + 1
                else:
                    child.g = current_node.g + math.sqrt(2)
                child.h = goal.distanceGoal(child.position)
                child.f = child.g + child.h
                if len([i for i in open_list if child == i and child.g >= i.g]) > 0:
                    continue
                heapq.heappush(open_list, child)

        solved = False
        approximate = False
        if not solution:
            solution = approxsol
            approximate = False
        if solution:
            path = og.PathGeometric(si)
            for s in self.states_[:solution]:
                path.append(s)
            pdef.addSolutionPath(path)
            solved = True
        return ob.PlannerStatus(solved, approximate)

    def clear(self):
        super(Astar, self).clear()
        self.states_ = []
