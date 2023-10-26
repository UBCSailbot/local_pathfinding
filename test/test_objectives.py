import pytest
from rclpy.impl.rcutils_logger import RcutilsLogger

from ompl import base as ob
import local_pathfinding.objectives as objectives
from local_pathfinding.objectives import (
    bound_to_180,
    is_angle_between,
)
import math  # for math.degrees()
import local_pathfinding.ompl_path as ompl_path
import local_pathfinding.local_path as local_path


# Upwind downwind cost multipliers
UPWIND_MULTIPLIER = 3000.0
DOWNWIND_MULTIPLIER = 3000.0


PATH = ompl_path.OMPLPath(
    parent_logger=RcutilsLogger(),
    max_runtime=1,
    local_path_state=None,  # type: ignore[arg-type] # None is placeholder
)


def test_distance_objective():
    distance_objective = objectives.DistanceObjective(PATH._simple_setup.getSpaceInformation())
    assert distance_objective is not None


def test_get_path_length_objective():
    dist_object = objectives.DistanceObjective(PATH._simple_setup.getSpaceInformation())

    assert dist_object.get_ompl_path_length_objective() != None


@pytest.mark.parametrize(
    "cs1,cs2,expected",
    [
        ((0, 0), (0, 0), 0),
        ((0.5, 0.5), (0.1, 0.2), 0.5),
    ],
)
def test_get_euclidean_path_length_objective(cs1, cs2, expected):
    space = ob.SE2StateSpace()

    s1 = ob.State(space)
    s1().setXY(cs1[0], cs1[1])

    s2 = ob.State(space)
    s2().setXY(cs2[0], cs2[1])

    dist_object = objectives.DistanceObjective(PATH._simple_setup.getSpaceInformation())

    assert dist_object.get_euclidean_path_length_objective(s1(), s2()) == expected


@pytest.mark.parametrize(
    "cs1,cs2,expected",
    [
        ((0, 0), (0, 0), 0),
        ((13.205724, 29.828011), (13.205824, 29.828111), 0.01552),
    ],
)
def test_get_latlon_path_length_objective(cs1, cs2, expected):
    space = ob.SE2StateSpace()

    s1 = ob.State(space)
    s1().setXY(cs1[0], cs1[1])

    s2 = ob.State(space)
    s2().setXY(cs2[0], cs2[1])

    dist_object = objectives.DistanceObjective(PATH._simple_setup.getSpaceInformation())

    assert round(dist_object.get_latlon_path_length_objective(s1(), s2()), 2) == round(expected, 2)


def test_minimum_turning_objective():
    minimum_turning_objective = objectives.MinimumTurningObjective(
        PATH._simple_setup.getSpaceInformation(), PATH._simple_setup, PATH.state.headingDirection
    )
    assert minimum_turning_objective is not None


@pytest.mark.parametrize(
    "cs1,cs2,sf,expected",
    [
        ((0, 0), (0, 0), (0, 0), 0),
        ((-1, -1), (2, 1), (0.1, 0.2), 13.799),
    ],
)
def test_goalPathTurnCost(cs1: tuple, cs2: tuple, sf: tuple, expected: float):
    space = ob.SE2StateSpace()

    goal = ob.State(space)

    s1 = ob.State(space)
    s2 = ob.State(space)

    goal().setXY(sf[0], sf[1])
    PATH._simple_setup.setGoalState(goal)

    s1().setXY(cs1[0], cs1[1])
    s2().setXY(cs2[0], cs2[1])

    minimum_turning_objective = objectives.MinimumTurningObjective(
        PATH._simple_setup.getSpaceInformation(), PATH._simple_setup, PATH.state.headingDirection
    )
    assert minimum_turning_objective.goalPathTurnCost(s1(), s2()) == pytest.approx(
        expected, abs=1e-3
    )


@pytest.mark.parametrize(
    "cs1,sf,heading,expected",
    [
        ((0, 0), (0, 0), 0, 0),
        ((-1, -1), (0.1, 0.2), 45, 2.490),
    ],
)
def test_goalHeadingTurnCost(cs1: tuple, sf: tuple, heading: float, expected: float):
    space = ob.SE2StateSpace()

    goal = ob.State(space)

    s1 = ob.State(space)

    goal().setXY(sf[0], sf[1])
    PATH._simple_setup.setGoalState(goal)

    s1().setXY(cs1[0], cs1[1])

    PATH.state.headingDirection = heading

    minimum_turning_objective = objectives.MinimumTurningObjective(
        PATH._simple_setup.getSpaceInformation(), PATH._simple_setup, PATH.state.headingDirection
    )
    assert minimum_turning_objective.goalHeadingTurnCost(s1()) == pytest.approx(expected, abs=1e-3)


@pytest.mark.parametrize(
    "cs1,cs2,heading,expected",
    [
        ((0, 0), (0, 0), 0.0, 0),
        ((-1, -1), (2, 1), 45.0, 11.310),
    ],
)
def test_headingPathTurnCost(cs1: tuple, cs2: tuple, heading: float, expected: float):
    space = ob.SE2StateSpace()

    s1 = ob.State(space)
    s2 = ob.State(space)

    s1().setXY(cs1[0], cs1[1])
    s2().setXY(cs2[0], cs2[1])

    PATH.state.headingDirection = heading

    minimum_turning_objective = objectives.MinimumTurningObjective(
        PATH._simple_setup.getSpaceInformation(), PATH._simple_setup, PATH.state.headingDirection
    )
    assert minimum_turning_objective.headingPathTurnCost(s1(), s2()) == pytest.approx(
        expected, abs=1e-3
    )


@pytest.mark.parametrize(
    "cs1,cs2,windDirection,expected",
    [
        ((0, 0), (0, 0), 0.0, 0 * UPWIND_MULTIPLIER),
        ((-1, -1), (2, 1), 45.0, 3.605551275 * UPWIND_MULTIPLIER),
    ],
)
def test_wind_objective(cs1: tuple, cs2: tuple, windDirection: float, expected: float):
    space = ob.SE2StateSpace()

    s1 = ob.State(space)
    s2 = ob.State(space)

    s1().setXY(cs1[0], cs1[1])
    s2().setXY(cs2[0], cs2[1])

    PATH.state.windDirection = windDirection

    wind_objective = objectives.WindObjective(
        PATH._simple_setup.getSpaceInformation(), PATH.state.windDirection
    )

    assert wind_objective.motionCost(s1(), s2()) == pytest.approx(expected, abs=1e-3)


@pytest.mark.parametrize(
    "windDirection,heading,expected",
    [
        (0, 0.0, True),
        (0.0, 45.0, False),
    ],
)
def test_isUpwind(windDirection: float, heading: float, expected: float):
    PATH.state.headingDirection = heading
    PATH.state.windDirection = windDirection

    assert (
        objectives.isUpwind(
            math.radians(PATH.state.windDirection), math.radians(PATH.state.headingDirection)
        )
        == expected
    )


@pytest.mark.parametrize(
    "windDirection,heading,expected",
    [
        (0.0, 0.0, 0),
        (25.0, 45.0, False),
    ],
)
def test_isDownwind(windDirection: float, heading: float, expected: float):
    PATH.state.headingDirection = heading
    PATH.state.windDirection = windDirection

    assert (
        objectives.isDownwind(
            math.radians(PATH.state.windDirection), math.radians(PATH.state.headingDirection)
        )
        == expected * DOWNWIND_MULTIPLIER
    )


""" Tests for bound_to_180() """


def test_bound_basic1():
    assert bound_to_180(0) == 0
    assert bound_to_180(-180) == -180
    assert bound_to_180(180) == -180
    assert bound_to_180(540) == -180
    assert bound_to_180(720) == 0
    assert bound_to_180(60) == 60
    assert bound_to_180(-45) == -45
    assert bound_to_180(120) == 120
    assert bound_to_180(-125) == -125
    assert bound_to_180(179) == 179
    assert bound_to_180(-179) == -179
    assert bound_to_180(-3779) == -179
    assert bound_to_180(3779) == 179
    assert bound_to_180(360) == 0


""" Tests for is_angle_between() """


def test_between_basic1():
    """Checks different situations such as boundary conditions.
    For example, what happens when par1 == par2 == par3? In addition, what happens if we change the order of the parameters
    """
    assert is_angle_between(0, 1, 2) == 1
    assert is_angle_between(0, 20, 360) == 0
    assert is_angle_between(-20, 10, 40) == 1
    assert is_angle_between(0, 30, 60) == 1

    # Checks negative angle conditions such as can it identify a negative angle between a positive and negative angle.
    assert is_angle_between(-170, -130, -90) == 1
    assert is_angle_between(-170, -130, 100) == 0
