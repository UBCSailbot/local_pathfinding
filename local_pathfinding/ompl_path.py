from ompl import base as ob
from ompl import geometric as og
from ompl import util as ou
from rclpy.impl.rcutils_logger import RcutilsLogger


class OMPLPath:
    def __init__(self, parent_logger: RcutilsLogger):
        self.logger = parent_logger.get_child(name='ompl_path')
        # self.ref_latlon = None
        self.simple_setup = None

        # only log OMPL warnings and above
        ou.setLogLevel(ou.LOG_WARN)

    def get_cost(self):
        pass

    def update_objectives(self):
        pass

    def plan(self):
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
        self.logger.debug(
            'state space bounds: '
            f'x=[{bounds.low[0]}, {bounds.high[0]}]; '
            f'y=[{bounds.low[1]}, {bounds.high[1]}]'
        )
        bounds.check()  # check if bounds are valid
        space.setBounds(bounds)

        # create a simple setup object
        ss = og.SimpleSetup(space)
        ss.setStateValidityChecker(ob.StateValidityCheckerFn(is_state_valid))

        # set the goal and start states of the simple setup object
        start = ob.State(space)
        goal = ob.State(space)
        start().setXY(x=0.5, y=0.4)
        goal().setXY(x=-0.5, y=-0.4)
        self.logger.debug(
            'start and goal state: '
            f'start=({start().getX()}, {start().getY()}); '
            f'goal=({goal().getX()}, {goal().getY()})'
        )
        ss.setStartAndGoalStates(start, goal)

        # set the optimization objective of the simple setup object
        # TODO: implement and add optimization objective here
        # ss.setOptimizationObjective(objective)

        # set the planner of the simple setup object
        # TODO: implement and add planner here
        # ss.setPlanner(planner)

        # this will automatically choose a default planner with default parameters
        solved = ss.solve(time=1.0)  # time is in seconds
        if not solved:
            raise RuntimeError('Could not find solution path')

        # try to shorten the path
        ss.simplifySolution()

        # update attributes
        self.simple_setup = ss


def is_state_valid(state: ob.SE2StateSpace) -> bool:
    # note: `state` is of type `SE2StateInternal`, so we don't need to use the `()` operator.
    return state.getX() < 0.6
