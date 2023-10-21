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


def test_get_path_length_objective(s1: tuple, s2: float):
    space = ob.SE2StateSpace()

    s1 = ob.State(space)
    s1().setXY(0.5, 0.5)

    s2 = ob.State(space)
    s2().setXY(0.1, 0.2)

    dist_object = objectives.DistanceObjective(PATH._simple_setup.getSpaceInformation())

    assert dist_object.get_ompl_path_length_objective() == 0


@pytest.mark.parametrize(
    "s1,s2,expected",
    [
        ((0, 0), (0, 0), 0),
        ((0.5, 0.5), (0.1, 0.2), 0.5),
    ],
)
def test_get_euclidean_path_length_objective(s1, s2, expected):
    space = ob.SE2StateSpace()

    s1 = ob.State(space)
    s1().setXY(s1[0], s2[1])

    s2 = ob.State(space)
    s2().setXY(s2[0], s2[1])

    dist_object = objectives.DistanceObjective(PATH._simple_setup.getSpaceInformation())

    assert dist_object.get_ompl_path_length_objective(s1, s2) == expected


@pytest.mark.parametrize(
    "x,y,expected",
    [
        ((0, 0), (0, 0), 0),
        ((13.205724, 29.828011), (13.205824, 29.828111), 0.01552),
    ],
)
def test_get_latlon_path_length_objective(s1, s2, expected):
    space = ob.SE2StateSpace()

    s1 = ob.State(space)
    s1().setXY(s1[0], s1[1])

    s2 = ob.State(space)
    s2().setXY(s2[0], s2[1])

    dist_object = objectives.DistanceObjective(PATH._simple_setup.getSpaceInformation())

    assert dist_object.get_ompl_path_length_objective(s1, s2) == expected


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
