"""The path to the next global waypoint, represented by the `LocalPath` class."""

from custom_interfaces.msg import GPS, AISShips, GlobalPath, WindSensor
from rclpy.impl.rcutils_logger import RcutilsLogger

from local_pathfinding.ompl_path import OMPLPath


class LocalPathState:
    """Gathers and stores the state of the Sailbot.

    Attributes:
        `postion` (GPS): The latitude (float32) and longitudinal (float32) coordinates of the Sailbot.
        `speed` (GPS): The speed (float32) of the Sailbot at that position. Units: km/hr.
        `heading` (GPS): The direction (float32) in which the Sailbot is Sailing at. Units: Degrees.
        `ais_ships` (AISShips): List of ships.
        `global_path` (GlobalPath): Objects of all the global way points which the Sailbot will travel to.
        `wind_speed` (WindSensor): The wind speed. Units: km/hr.
        `wind_direction` (WindSensor): The wind direction (int16) towards the boat. Units: Degrees.
    """

    def __init__(
        self,
        gps: GPS,
        ais_ships: AISShips,
        global_path: GlobalPath,
        filtered_wind_sensor: WindSensor,
    ):
        """Initializes the local path state.

        Args:
            `gps` (GPS): Data from the GPS sensors.
            `ais_ships` (AISShips): Data from the AIS receiver.
            `global_path` (GlobalPath): Data from the Globalpath server.
            `filtered_wind_sensor` (WindSensor): Data from the windsensors.
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
            `gps` (GPS): Data from the GPS sensors.
            `ais_ships` (AISShips): Data from the AIS receiver.
            `global_path` (GlobalPath): Data from the Globalpath server.
            `filtered_wind_sensor` (WindSensor): Data from the windsensors.
        """
        state = LocalPathState(gps, ais_ships, global_path, filtered_wind_sensor)
        ompl_path = OMPLPath(parent_logger=self._logger, max_runtime=1.0, local_path_state=state)
        if ompl_path.solved:
            self._logger.info('Updating local path')
            self._update(ompl_path)

    def _update(self, ompl_path: OMPLPath):
        self._ompl_path = ompl_path
        self.waypoints = self._ompl_path.get_waypoints()
