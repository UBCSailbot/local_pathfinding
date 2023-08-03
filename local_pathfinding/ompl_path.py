"""The local_pathfinding<->OMPL interface, represented by the OMPLPath class.

OMPL is written in C++, but Python bindings were generated to interface with OMPL in Python.
VS Code currently can't read these bindings, so LSP features (autocomplete, go to definition, etc.
won't work). The C++ API is documented on the OMPL website:
https://ompl.kavrakilab.org/api_overview.html.
"""
from __future__ import annotations

from typing import TYPE_CHECKING

from ompl import base as ob
from ompl import geometric as og
from ompl import util as ou
from rclpy.impl.rcutils_logger import RcutilsLogger

from local_pathfinding.path_objective import Distanceobjective

if TYPE_CHECKING:
    from local_pathfinding.local_path import LocalPathState

# OMPL logging: only log warnings and above
ou.setLogLevel(ou.LOG_WARN)


class OMPLPathState:
    def __init__(self, local_path_state: LocalPathState):
        # TODO: derive OMPLPathState attributes from local_path_state
        self.state_domain = (-0.6, 0.6)
        self.state_range = (-0.6, 0.6)
        self.start_state = (0, 0)
        self.goal_state = (0.5, -0.4)


class OMPLPath:
    def __init__(
        self,
        parent_logger: RcutilsLogger,
        max_runtime: float,
        local_path_state: LocalPathState,
    ):
        self._logger = parent_logger.get_child(name='ompl_path')
        self.state = OMPLPathState(local_path_state)
        self._simple_setup = self._init_simple_setup()
        self.solved = self._simple_setup.solve(time=max_runtime)  # time is in seconds

        # TODO: play around with simplifySolution()
        # if self.solved:
        #     # try to shorten the path
        #     simple_setup.simplifySolution()

    def get_cost(self):
        raise NotImplementedError

    def get_waypoints(self):
        if not self.solved:
            self._logger.warn('Trying to get the waypoints of an unsolved OMPLPath')
            return []

        solution_path = self._simple_setup.getSolutionPath()
        waypoints = [(state.getX(), state.getY()) for state in solution_path.getStates()]
        return waypoints

    def update_objectives(self, space_information):
        distance_ob = Distanceobjective(space_information)

        distance_objective = distance_ob.get_path_length_objective()
        # euclidean_objective = distance_ob.get_euclidean_path_length_objective()

        print(distance_objective)

        return distance_objective


    def _init_simple_setup(self) -> og.SimpleSetup:
        # create an SE2 state space: rotation and translation in a plane
        space = ob.SE2StateSpace()

        # set the bounds of the state space
        bounds = ob.RealVectorBounds(dim=2)
        x_min, x_max = self.state.state_domain
        y_min, y_max = self.state.state_range
        bounds.setLow(index=0, value=x_min)
        bounds.setLow(index=1, value=y_min)
        bounds.setHigh(index=0, value=x_max)
        bounds.setHigh(index=1, value=y_max)
        self._logger.debug(
            'state space bounds: '
            f'x=[{bounds.low[0]}, {bounds.high[0]}]; '
            f'y=[{bounds.low[1]}, {bounds.high[1]}]'
        )
        bounds.check()  # check if bounds are valid
        space.setBounds(bounds)

        # create a simple setup object
        simple_setup = og.SimpleSetup(space)
        simple_setup.setStateValidityChecker(ob.StateValidityCheckerFn(is_state_valid))

        # Constructs a space information instance for this simple setup
        space_information = simple_setup.getSpaceInformation()

        # set the goal and start states of the simple setup object
        start = ob.State(space)
        goal = ob.State(space)
        start_x, start_y = self.state.start_state
        goal_x, goal_y = self.state.goal_state
        start().setXY(start_x, start_y)
        goal().setXY(goal_x, goal_y)
        self._logger.debug(
            'start and goal state: '
            f'start=({start().getX()}, {start().getY()}); '
            f'goal=({goal().getX()}, {goal().getY()})'
        )
        simple_setup.setStartAndGoalStates(start, goal)

        # set the optimization objective of the simple setup object
        # TODO: implement and add optimization objective here
        objective = self.update_objectives(space_information)
        simple_setup.setOptimizationObjective(objective)



        # set the planner of the simple setup object
        # TODO: implement and add planner here
        planner = og.RRTstar(space_information)
        simple_setup.setPlanner(planner)

        solve = simple_setup.solve(20)

        if solve:
            with open("path.txt", "w") as f:
                f.write(simple_setup.getSolutionPath().printAsMatrix())
                f.close()

            # if ss is a ompl::geometric::SimpleSetup object
            print(simple_setup.getSolutionPath().printAsMatrix())


        return simple_setup


def is_state_valid(state: ob.SE2StateSpace) -> bool:
    # TODO: implement obstacle avoidance here
    # note: `state` is of type `SE2StateInternal`, so we don't need to use the `()` operator.
    return state.getX() < 0.6
