"""The local_pathfinding<->OMPL interface, represented by the OMPLPath class.

OMPL is written in C++, but Python bindings were generated to interface with OMPL in Python.
VS Code currently can't read these bindings, so LSP features (autocomplete, go to definition, etc.
won't work). The C++ API is documented on the OMPL website:
https://ompl.kavrakilab.org/api_overview.html.
"""

from ompl import base as ob
from ompl import geometric as og
from ompl import util as ou
from rclpy.impl.rcutils_logger import RcutilsLogger

# OMPL logging: only log warnings and above
ou.setLogLevel(ou.LOG_WARN)


class OMPLPath:
    """Represents the general OMPL Path"""

    def __init__(self, parent_logger: RcutilsLogger, max_runtime: float):
        """Initialize the OMPL Path Class

        Args:
            parent_logger (RcutilsLogger): Represents the logger of the parent class
            max_runtime (float): Represents the maximum runtime to solve for a path
        """
        self._logger = parent_logger.get_child(name='ompl_path')
        self._simple_setup = self._init_simple_setup()
        self.solved = self._simple_setup.solve(time=max_runtime)  # time is in seconds

        # TODO: play around with simplifySolution()
        # if self.solved:
        #     # try to shorten the path
        #     simple_setup.simplifySolution()

    def get_cost(self):
        """Returns the cost of the path generated

        Raises:
            NotImplementedError: Method or function hasn't been implemented yet
        """
        raise NotImplementedError

    def get_waypoints(self) -> list[tuple[float, float]]:
        """Returns a list of waypoints for the boat to follow

        Returns:
            list: A list of tuples representing the x and y coordinates of the waypoints
        """
        if not self.solved:
            self._logger.warn('Trying to get the waypoints of an unsolved OMPLPath')
            return []

        waypoints = []
        solution_path = self._simple_setup.getSolutionPath()
        waypoints = [(state.getX(), state.getY()) for state in solution_path.getStates()]
        return waypoints

    def update_objectives(self):
        """Updates the objectives on the basis of which the path is optimized

        Raises:
            NotImplementedError: Method or function hasn't been implemented yet
        """
        raise NotImplementedError

    def _init_simple_setup(self) -> og.SimpleSetup:
        """Initializes and configures the OMPL SimpleSetup object

        Returns:
            og.SimpleSetup: A configured SimpleSetup object based on the problem
        """
        # create an SE2 state space: rotation and translation in a plane
        space = ob.SE2StateSpace()

        # set the bounds of the state space
        bounds = ob.RealVectorBounds(dim=2)
        x_min, x_max = -1, 1
        y_min, y_max = -1, 1
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

        # set the goal and start states of the simple setup object
        start = ob.State(space)
        goal = ob.State(space)
        start().setXY(x=0.5, y=0.4)
        goal().setXY(x=-0.5, y=-0.4)
        self._logger.debug(
            'start and goal state: '
            f'start=({start().getX()}, {start().getY()}); '
            f'goal=({goal().getX()}, {goal().getY()})'
        )
        simple_setup.setStartAndGoalStates(start, goal)

        # set the optimization objective of the simple setup object
        # TODO: implement and add optimization objective here
        # simple_setup.setOptimizationObjective(objective)

        # set the planner of the simple setup object
        # TODO: implement and add planner here
        # simple_setup.setPlanner(planner)

        return simple_setup


def is_state_valid(state: ob.SE2StateSpace) -> bool:
    """Checks if the state is valid

    Args:
        state (ob.SE2StateSpace): State to check

    Returns:
        bool: True if state is valid else False
    """
    # TODO: implement obstacle avoidance here
    # note: `state` is of type `SE2StateInternal`, so we don't need to use the `()` operator.
    return state.getX() < 0.6
