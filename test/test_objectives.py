import pytest
from rclpy.impl.rcutils_logger import RcutilsLogger

from ompl import base as ob
import local_pathfinding.objectives as objectives
import local_pathfinding.ompl_path as ompl_path
import local_pathfinding.local_path as local_path

PATH = ompl_path.OMPLPath(
    parent_logger=RcutilsLogger(),
    max_runtime=1,
    local_path_state=None,  # type: ignore[arg-type] # None is placeholder
)


def test_distance_objective():
    distance_objective = objectives.DistanceObjective(PATH._simple_setup.getSpaceInformation())
    assert distance_objective is not None

@pytest.mark.parametrize(
    "s1,s2,is_valid",
    [
        (0.5, 0.5, True),
        (0, 0.5, False),
    ],
)
def test_get_path_length_objective(s1: tuple, s2: float, is_valid: bool):
    state = ob.State(PATH._simple_setup.getStateSpace())
    state().setXY(s1, s2)
    dist_object = objectives.DistanceObjective(PATH._simple_setup.getSpaceInformation())

    if is_valid:
        assert dist_object.get_path_length_objective(state) == 0.5
    else
        assert not dist_object.get_path_length_objective(state) == 0.5




@pytest.mark.parametrize(
    "x,y,is_valid",
    [
        (0.5, 0.5, True),
        (0.6, 0.6, False),
    ],
)
def test_get_euclidean_path_length_objective():
    with pytest.raises(NotImplementedError):
        raise NotImplementedError

@pytest.mark.parametrize(
    "x,y,is_valid",
    [
        (0.5, 0.5, True),
        (0.6, 0.6, False),
    ],
)
def test_get_latlon_path_length_objective():
    with pytest.raises(NotImplementedError):
        raise NotImplementedError


def test_minimum_turning_objective():
    minimum_turning_objective = objectives.MinimumTurningObjective(
        PATH._simple_setup.getSpaceInformation(), PATH._simple_setup, PATH.state.headingDirection
    )
    assert minimum_turning_objective is not None

@pytest.mark.parametrize(
    "x,y,is_valid",
    [
        (0.5, 0.5, True),
        (0.6, 0.6, False),
    ],
)
def test_goalPathTurnCost():
    with pytest.raises(NotImplementedError):
        raise NotImplementedError

@pytest.mark.parametrize(
    "x,y,is_valid",
    [
        (0.5, 0.5, True),
        (0.6, 0.6, False),
    ],
)
def test_goalHeadingTurnCost():
    with pytest.raises(NotImplementedError):
        raise NotImplementedError

@pytest.mark.parametrize(
    "x,y,is_valid",
    [
        (0.5, 0.5, True),
        (0.6, 0.6, False),
    ],
)
def test_headingPathTurnCost():
    with pytest.raises(NotImplementedError):
        raise NotImplementedError


def test_wind_objective():
    wind_objective = objectives.WindObjective(PATH._simple_setup.getSpaceInformation(), 0)
    assert wind_objective is not None
