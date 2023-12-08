# import itertools
import math

import pytest

# from ompl import base as ob
from rclpy.impl.rcutils_logger import RcutilsLogger

import local_pathfinding.coord_systems as coord_systems
import local_pathfinding.objectives as objectives
import local_pathfinding.ompl_path as ompl_path

# Upwind downwind cost multipliers
UPWIND_MULTIPLIER = 3000.0
DOWNWIND_MULTIPLIER = 3000.0


PATH = ompl_path.OMPLPath(
    parent_logger=RcutilsLogger(),
    max_runtime=1,
    local_path_state=None,  # type: ignore[arg-type] # None is placeholder
)


@pytest.mark.parametrize(
    "method,max_motion_cost",
    [
        (objectives.DistanceMethod.EUCLIDEAN, 2.5),
        (objectives.DistanceMethod.LATLON, 2700),
        (objectives.DistanceMethod.OMPL_PATH_LENGTH, 4.0),
    ],
)
def test_distance_objective(method: objectives.DistanceMethod, max_motion_cost: float):
    distance_objective = objectives.DistanceObjective(
        PATH._simple_setup.getSpaceInformation(),
        method,
    )

    # test sample_states()
    num_samples = 3
    sampled_states = distance_objective.sample_states(num_samples)
    assert len(sampled_states) == num_samples
    # for state in sampled_states:
    #     assert ompl_path.is_state_valid(state)

    # test find_maximum_motion_cost()
    # implicitly
    assert distance_objective.max_motion_cost == pytest.approx(max_motion_cost, rel=1e0)
    # explicitly for the easiest method to set up
    # don't need to test for all methods since they each have their own tests
    # if method == objectives.DistanceMethod.OMPL_PATH_LENGTH:
    #     states = []
    #     for xy in [(-0.5, 0.4), (0.1, 0.2), (0.3, -0.6)]:
    #         state = ob.State(distance_objective.space_information)
    #         state().setXY(*xy)
    #         states.append(state)
    #     assert type(states[0]) is type(sampled_states[0]), "states are not the correct type"
    #     costs = [
    #         distance_objective.ompl_path_objective.motionCost(s1(), s2()).value()
    #         for s1, s2 in itertools.combinations(iterable=states, r=2)
    #     ]
    #     assert distance_objective.find_maximum_motion_cost(states) == pytest.approx(max(costs))


@pytest.mark.parametrize(
    "cs1,cs2,expected",
    [
        ((0, 0), (0, 0), 0),
        ((0.5, 0.5), (0.1, 0.2), 0.5),
    ],
)
def test_get_euclidean_path_length_objective(cs1: tuple, cs2: tuple, expected: float):
    s1 = coord_systems.XY(*cs1)
    s2 = coord_systems.XY(*cs2)
    assert objectives.DistanceObjective.get_euclidean_path_length_objective(s1, s2) == expected


@pytest.mark.parametrize(
    "rf, cs1,cs2",
    [
        ((10, 10), (0, 0), (0, 0)),
        ((13.206724, 29.829011), (13.208724, 29.827011), (13.216724, 29.839011)),
        ((0, 0), (0, 0.1), (0, -0.1)),
        ((0, 0), (0.1, 0), (-0.1, 0)),
        ((0, 0), (0.1, 0.1), (-0.1, -0.1)),
    ],
)
def test_get_latlon_path_length_objective(rf: tuple, cs1: tuple, cs2: tuple):
    reference = coord_systems.LatLon(*rf)
    s1 = coord_systems.LatLon(*cs1)
    s2 = coord_systems.LatLon(*cs2)
    ls1 = coord_systems.latlon_to_xy(reference, s1)
    ls2 = coord_systems.latlon_to_xy(reference, s2)
    _, _, distance_m = coord_systems.GEODESIC.inv(
        lats1=s1.latitude,
        lons1=s1.longitude,
        lats2=s2.latitude,
        lons2=s2.longitude,
    )

    assert objectives.DistanceObjective.get_latlon_path_length_objective(
        ls1,
        ls2,
        reference,
    ) == pytest.approx(distance_m)


@pytest.mark.parametrize(
    "method",
    [
        objectives.MinimumTurningMethod.GOAL_HEADING,
        objectives.MinimumTurningMethod.GOAL_PATH,
        objectives.MinimumTurningMethod.HEADING_PATH,
    ],
)
def test_minimum_turning_objective(method: objectives.MinimumTurningMethod):
    minimum_turning_objective = objectives.MinimumTurningObjective(
        PATH._simple_setup.getSpaceInformation(),
        PATH._simple_setup,
        PATH.state.heading_direction,
        method,
    )
    assert minimum_turning_objective is not None


@pytest.mark.parametrize(
    "cs1,sf,heading_degrees,expected",
    [
        ((0, 0), (0, 0), 0, 0),
        ((-1, -1), (0.1, 0.2), 45, 2.490),
    ],
)
def test_goal_heading_turn_cost(cs1: tuple, sf: tuple, heading_degrees: float, expected: float):
    s1 = coord_systems.XY(*cs1)
    goal = coord_systems.XY(*sf)
    heading = math.radians(heading_degrees)
    assert objectives.MinimumTurningObjective.goal_heading_turn_cost(
        s1, goal, heading
    ) == pytest.approx(expected, abs=1e-3)


@pytest.mark.parametrize(
    "cs1,cs2,sf,expected",
    [
        ((0, 0), (0, 0), (0, 0), 0),
        ((-1, -1), (2, 1), (0.1, 0.2), 13.799),
    ],
)
def test_goal_path_turn_cost(cs1: tuple, cs2: tuple, sf: tuple, expected: float):
    s1 = coord_systems.XY(*cs1)
    s2 = coord_systems.XY(*cs2)
    goal = coord_systems.XY(*sf)

    assert objectives.MinimumTurningObjective.goal_path_turn_cost(s1, s2, goal) == pytest.approx(
        expected, abs=1e-3
    )


@pytest.mark.parametrize(
    "cs1,cs2,heading_degrees,expected",
    [
        ((0, 0), (0, 0), 0.0, 0),
        ((-1, -1), (2, 1), 45.0, 11.310),
    ],
)
def test_heading_path_turn_cost(cs1: tuple, cs2: tuple, heading_degrees: float, expected: float):
    s1 = coord_systems.XY(*cs1)
    s2 = coord_systems.XY(*cs2)
    heading = math.radians(heading_degrees)

    assert objectives.MinimumTurningObjective.heading_path_turn_cost(
        s1, s2, heading
    ) == pytest.approx(expected, abs=1e-3)


@pytest.mark.parametrize(
    "cs1,cs2,wind_direction_deg,expected",
    [
        ((0, 0), (0, 0), 0.0, 0 * UPWIND_MULTIPLIER),
        ((-1, -1), (2, 1), 45.0, 3.605551275 * UPWIND_MULTIPLIER),
    ],
)
def test_wind_direction_cost(cs1: tuple, cs2: tuple, wind_direction_deg: float, expected: float):
    s1 = coord_systems.XY(*cs1)
    s2 = coord_systems.XY(*cs2)
    wind_direction = math.radians(wind_direction_deg)
    assert objectives.WindObjective.wind_direction_cost(s1, s2, wind_direction) == pytest.approx(
        expected, abs=1e-3
    )


@pytest.mark.parametrize(
    "wind_direction_deg,heading_deg,expected",
    [
        (0, 0.0, True),
        (0.0, 45.0, False),
    ],
)
def test_is_upwind(wind_direction_deg: float, heading_deg: float, expected: float):
    wind_direction = math.radians(wind_direction_deg)
    heading = math.radians(heading_deg)

    assert objectives.WindObjective.is_upwind(wind_direction, heading) == expected


@pytest.mark.parametrize(
    "wind_direction_deg,heading_deg,expected",
    [
        (0.0, 0.0, False),
        (25.0, 46.0, False),
        (0, 180, True),
        (225, 45, True),
    ],
)
def test_is_downwind(wind_direction_deg: float, heading_deg: float, expected: float):
    wind_direction = math.radians(wind_direction_deg)
    heading = math.radians(heading_deg)

    assert objectives.WindObjective.is_downwind(wind_direction, heading) == expected


""" Tests for is_angle_between() """


@pytest.mark.parametrize(
    "afir,amid,asec,expected",
    [
        (0, 1, 2, 1),
        (0, 20, 360, 0),
        (-20, 10, 40, 1),
        (0, 30, 60, 1),
        (-170, -130, -90, 1),
        (-170, -130, 100, 0),
        (400, 410, 420, 1),
        (400, 420, 410, 0),
        (370, 0, -370, 1),
        (370, 15, -370, 0),
        (-90, 270, 450, 0),
    ],
)
def test_angle_between(afir: float, amid: float, asec: float, expected: float):
    """Checks different situations such as boundary conditions.
    For example, what happens when par1 == par2 == par3?
    In addition, what happens if we change the order of the parameters
    """

    assert (
        objectives.WindObjective.is_angle_between(
            math.radians(afir), math.radians(amid), math.radians(asec)
        )
        == expected
    )
