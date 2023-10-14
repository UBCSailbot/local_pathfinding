import pytest
from ompl import base as ob
from rclpy.impl.rcutils_logger import RcutilsLogger

import local_pathfinding.ompl_path as ompl_path

PATH = ompl_path.OMPLPath(
    parent_logger=RcutilsLogger(),
    max_runtime=1,
    local_path_state=None,  # type: ignore[arg-type] # None is placeholder
)


def test_OMPLPathState():
    state = ompl_path.OMPLPathState(local_path_state=None)
    assert state.state_domain == (-1, 1), "incorrect value for attribute state_domain"
    assert state.state_range == (-1, 1), "incorrect value for attribute start_state"
    assert state.start_state == pytest.approx(
        (0.5, 0.4)
    ), "incorrect value for attribute start_state"
    assert state.goal_state == pytest.approx(
        (0.5, -0.4)
    ), "incorrect value for attribute goal_state"


def test_OMPLPath___init__():
    assert PATH.solved


def test_OMPLPath_get_cost():
    with pytest.raises(NotImplementedError):
        PATH.get_cost()


def test_OMPLPath_get_waypoint():
    waypoints = PATH.get_waypoints()
    assert waypoints[0] == pytest.approx(
        PATH.state.start_state
    ), "first waypoint should be start state"
    assert waypoints[-1] == pytest.approx(
        PATH.state.goal_state
    ), "last waypoint should be goal state"


def test_OMPLPath_update_objectives():
    PATH.update_objectives(
        PATH._simple_setup.getSpaceInformation(),
        PATH._simple_setup,
        heading_degrees=45,
        windDirectionDegrees=75,
    )
    assert PATH._simple_setup.getOptimizationObjective() is not None


@pytest.mark.parametrize(
    "x,y,is_valid",
    [
        (0.5, 0.5, True),
        (0.6, 0.6, False),
    ],
)
def test_is_state_valid(x: float, y: float, is_valid: bool):
    state = ob.State(PATH._simple_setup.getStateSpace())
    state().setXY(x, y)

    if is_valid:
        assert ompl_path.is_state_valid(state()), "state should be valid"
    else:
        assert not ompl_path.is_state_valid(state()), "state should not be valid"
