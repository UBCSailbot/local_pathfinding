"""The path to the next global waypoint, represented by the `LocalPath` class."""

from typing import List, Optional, Tuple

from custom_interfaces.msg import GPS, AISShips, HelperLatLon, Path, WindSensor
from pyproj import Geod
from rclpy.impl.rcutils_logger import RcutilsLogger

from local_pathfinding.ompl_path import OMPLPath

GEODESIC = Geod(ellps="WGS84")


class LocalPathState:
    """Gathers and stores the state of Sailbot.
    The attributes' units and conventions can be found in the ROS msgs they are derived from in the
    custom_interfaces repository.

    Attributes:
        `position` (Tuple[float, float]): Latitude and longitude of Sailbot.
        `speed` (float): Speed of Sailbot.
        `heading` (float): Direction that Sailbot is pointing.
        `ais_ships` (List[HelperAISShip]): Information about nearby ships.
        `global_path` (List[Tuple[float, float]]): Path to the destination that Sailbot is
            navigating along.
        `wind_speed` (float): Wind speed.
        `wind_direction` (int): Wind direction.
    """

    def __init__(
        self,
        gps: GPS,
        ais_ships: AISShips,
        global_path: Path,
        filtered_wind_sensor: WindSensor,
    ):
        """Initializes the state from ROS msgs."""
        if gps:  # TODO: remove when mock can be run
            self.position = (gps.lat_lon.latitude, gps.lat_lon.longitude)
            self.speed = gps.speed.speed
            self.heading = gps.heading.heading

        if ais_ships:  # TODO: remove when mock can be run
            self.ais_ships = [ship for ship in ais_ships.ships]

        if global_path:  # TODO: remove when mock can be run
            self.global_path = [
                HelperLatLon(latitude=waypoint.latitude, longitude=waypoint.longitude)
                for waypoint in global_path.waypoints
            ]

        if filtered_wind_sensor:  # TODO: remove when mock can be run
            self.wind_speed = filtered_wind_sensor.speed.speed
            self.wind_direction = filtered_wind_sensor.direction


class LocalPath:
    """Sets and updates the OMPL path and the local waypoints

    Attributes:
        `_logger` (RcutilsLogger): ROS logger.
        `_ompl_path` (Optional[OMPLPath]): Raw representation of the path from OMPL.
        `waypoints` (Optional[List[Tuple[float, float]]]): List of coordinates that form the path
            to the next global waypoint.
    """

    def __init__(self, parent_logger: RcutilsLogger):
        self._logger = parent_logger.get_child(name="local_path")
        self._ompl_path: Optional[OMPLPath] = None
        self.waypoints: Optional[List[Tuple[float, float]]] = None
        # TODO Set the right max_runtime; set the thresholds
        # Should I check if any of the waypoint in global_path is changed?

        self.thresholds = {
            "speed_threshold": 1.0,
            "rot_threshold": 5.0,
            "heading_threshold": 10.0,
            "wind_speed_threshold": 1.0,
            "wind_direction_threshold": 10,
            "low_wind_threshold": 1.0,
            "gps_proximity_threshold": 100,  # meters
        }

    def update_if_needed(
        self,
        gps: GPS,
        ais_ships: AISShips,
        global_path: Path,
        filtered_wind_sensor: WindSensor,
    ):
        # Logic remains the same.
        if self._ompl_path is None:
            self._logger.info("No current OMPL path, updating path.")
            n_local_path_state = LocalPathState(gps, ais_ships, global_path, filtered_wind_sensor)
            self._update(n_local_path_state)
            return

        c_local_path_state = self._ompl_path.local_state
        n_local_path_state = LocalPathState(gps, ais_ships, global_path, filtered_wind_sensor)

        if filtered_wind_sensor < self.thresholds["low_wind_threshold"]:
            self._logger.info("Low wind speed condition: Not updating path.")
        else:
            # Check conditions and update path as needed.
            if self._check_ship_conditions(
                c_local_path_state.ais_ships, n_local_path_state.ais_ships
            ):
                self._logger.info("Significant AIS ship variation detected, updating path.")
                self._update(n_local_path_state)
                return

            if self._check_wind_conditions(
                c_local_path_state.wind_speed, n_local_path_state.wind_speed
            ):
                self._logger.info("Significant wind condition changes detected, updating path.")
                self._update(n_local_path_state)
                return

            if self._check_gps_on_global_waypoint(n_local_path_state):
                self._logger.info(
                    "GPS position is on or very close to a global waypoint, updating path."
                )
                self._update(n_local_path_state)
                return

    def _check_ship_conditions(self, c_ships: AISShips, n_ships: AISShips) -> bool:
        # TODO ask if the order of AISSship is consistent
        if len(c_ships) != len(n_ships):
            self._logger.info("Mismatch in the number of ships detected.")
            return True

        for c_ship, n_ship in zip(c_ships, n_ships):
            if abs(c_ship.sog.speed - n_ship.sog.speed) > self.thresholds["speed_threshold"]:
                self._logger.info(f"Speed variation threshold exceeded for ship ID {c_ship.id}.")
                return True
            if abs(c_ship.rot.rot - n_ship.rot.rot) > self.thresholds["rot_threshold"]:
                self._logger.info(f"ROT variation threshold exceeded for ship ID {c_ship.id}.")
                return True
            if abs(c_ship.cog.heading - n_ship.cog.heading) > self.thresholds["heading_threshold"]:
                self._logger.info(f"Heading variation threshold exceeded for ship ID {c_ship.id}.")
                return True
        return False

    def _check_wind_conditions(self, c_wind_sensor: WindSensor, n_wind_sensor: WindSensor) -> bool:
        return (
            abs(c_wind_sensor.speed - n_wind_sensor.speed)
            > self.thresholds["wind_speed_threshold"]
            or abs(c_wind_sensor.direction - n_wind_sensor.direction)
            > self.thresholds["wind_direction_threshold"]
        )

    def _check_gps_on_global_waypoint(self, n_local_path_state: LocalPathState) -> bool:
        if n_local_path_state.global_path and n_local_path_state.position:
            for waypoint in n_local_path_state.global_path:
                _, _, distance_to_waypoint = GEODESIC.inv(
                    n_local_path_state.position[1],
                    n_local_path_state.position[0],
                    waypoint.longitude,
                    waypoint.latitude,
                )
                if distance_to_waypoint < self.thresholds["gps_proximity_threshold"]:
                    self._logger.info("GPS is within proximity threshold of a global waypoint.")
                    return True
        return False

    def _update(self, n_local_path_state: LocalPathState):
        """Updates the OMPL path and waypoints based on the new local path state.

        Args:
            n_local_path_state (LocalPathState): The new LocalPathState
        """
        self._logger.info("Updating OMPL Path.")
        ompl_path = OMPLPath(self._logger, max_runtime=10.0, local_path_state=n_local_path_state)
        self._ompl_path = ompl_path
        self.waypoints = ompl_path.get_waypoints()
