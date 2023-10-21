import pytest
from rclpy.impl.rcutils_logger import RcutilsLogger

import local_pathfinding.objectives as objectives
import local_pathfinding.ompl_path as ompl_path

PATH = ompl_path.OMPLPath(
    parent_logger=RcutilsLogger(),
    max_runtime=1,
    local_path_state=None,  # type: ignore[arg-type] # None is placeholder
)


def test_distance_objective():
    distance_objective = objectives.DistanceObjective(PATH._simple_setup.getSpaceInformation())
    assert distance_objective is not None


def test_get_path_length_objective():
    with pytest.raises(NotImplementedError):
        raise NotImplementedError


def test_get_euclidean_path_length_objective():
    with pytest.raises(NotImplementedError):
        raise NotImplementedError


def test_get_latlon_path_length_objective():
    with pytest.raises(NotImplementedError):
        raise NotImplementedError


def test_minimum_turning_objective():
    minimum_turning_objective = objectives.MinimumTurningObjective(
        PATH._simple_setup.getSpaceInformation(), None, 0
    )
    assert minimum_turning_objective is not None


def test_goalPathTurnCost():
    with pytest.raises(NotImplementedError):
        raise NotImplementedError


def test_goalHeadingTurnCost():
    with pytest.raises(NotImplementedError):
        raise NotImplementedError


def test_headingPathTurnCost():
    with pytest.raises(NotImplementedError):
        raise NotImplementedError


def test_wind_objective():
    wind_objective = objectives.WindObjective(PATH._simple_setup.getSpaceInformation(), 0)
    assert wind_objective is not None
