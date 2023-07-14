"""The path to the next global waypoint, represented by the `LocalPath` class."""

from custom_interfaces.msg import GPS, AISShips, GlobalPath, WindSensor
from rclpy.impl.rcutils_logger import RcutilsLogger

from local_pathfinding.ompl_path import OMPLPath
from typing import List, Tuple


class LocalPathState:
    """Gathers and stores the state of the Sailbot.

    Attributes:
        `position` (Tuple[float, float]): The latitude and longitudinal coordinates of the
         Sailbot.
        `speed` (float): The speed of the Sailbot at that position.
        `heading` (float): The direction in which the Sailbot is Sailing at.
        `ais_ships` (List[HelperAISShip]): List of ships.
        `global_path` (List[Tuple[float, float]]): Objects of all the global way points which the Sailbot will
         travel to.
        `wind_speed` (float): The wind speed.
        `wind_direction` (int): The wind direction towards the boat.
        The attributes' units and conventions can be found in the ROS msgs, which are derived from
        custom_interfaces.
    """

    def __init__(
        self,
        gps: GPS,
        ais_ships: AISShips,
        global_path: GlobalPath,
        filtered_wind_sensor: WindSensor,
    ):
        """Initializes the local path finding state from the ROS msgs.

        Args:
            `gps`: The GPS msg.
            `ais_ships`: The AISShips msg.
            `global_path`: The GlobalPath msg.
            `filtered_wind_sensor`: The WindSensor msg.
        """
        if gps:  # TODO: remove when mock can be run
            self.position = (gps.lat_lon.latitude, gps.lat_lon.longitude)
            self.speed = gps.speed.speed_kmph
            self.heading = gps.heading.heading_degrees

        if ais_ships:  # TODO: remove when mock can be run
            self.ais_ships = [ship for ship in ais_ships.ships]


        if global_path:  # TODO: remove when mock can be run
            self.global_path = [
                (waypoint.latitude, waypoint.longitude) for waypoint in global_path.waypoints
            ]

        if filtered_wind_sensor:  # TODO: remove when mock can be run
            self.wind_speed = filtered_wind_sensor.speed.speed_kmph
            self.wind_direction = filtered_wind_sensor.direction_degrees


class LocalPath:
    """Sets and updates the OMPL path and the local waypoints

    Attributes:
        `_logger` (RcutilsLogger): The ROS logger of LocalPath.
        `_ompl_path` (OMPLPath): The raw representation of the path from OMPL.
        `waypoints` (List): A list of coordinates that form the path.
    """
    def __init__(self, parent_logger: RcutilsLogger):
        self._logger = parent_logger.get_child(name='local_path')
        self._ompl_path = None
        self.waypoints = None

    def update_if_needed(
        self,
        gps: GPS,
        ais_ships: AISShips,
        global_path: GlobalPath,
        filtered_wind_sensor: WindSensor,
    ):
        """Updates the LocalPath with an updated OMPLPath and new local waypoints.
           The path is updated if a new path is found.

        Args:
            `gps`: The GPS msg.
            `ais_ships`: The AISShips msg.
            `global_path`: The GlobalPath msg.
            `filtered_wind_sensor`: The WindSensor msg.
        """
        state = LocalPathState(gps, ais_ships, global_path, filtered_wind_sensor)
        ompl_path = OMPLPath(parent_logger=self._logger, max_runtime=1.0, local_path_state=state)
        if ompl_path.solved:
            self._logger.info('Updating local path')
            self._update(ompl_path)

    def _update(self, ompl_path: OMPLPath):
        self._ompl_path = ompl_path
        self.waypoints = self._ompl_path.get_waypoints()
