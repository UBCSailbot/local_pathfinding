"""The path to the next global waypoint, represented by the `LocalPath` class."""

from custom_interfaces.msg import GPS, AISShips, GlobalPath, WindSensor
from rclpy.impl.rcutils_logger import RcutilsLogger

from local_pathfinding.ompl_path import OMPLPath


class LocalPathState:
    def __init__(
        self,
        gps: GPS,
        ais_ships: AISShips,
        global_path: GlobalPath,
        filtered_wind_sensor: WindSensor,
    ):
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
        state = LocalPathState(gps, ais_ships, global_path, filtered_wind_sensor)
        ompl_path = OMPLPath(parent_logger=self._logger, max_runtime=1.0, local_path_state=state)
        if ompl_path.solved:
            self._logger.info('Updating local path')
            self._update(ompl_path)

    def _update(self, ompl_path: OMPLPath):
        self._ompl_path = ompl_path
        self.waypoints = self._ompl_path.get_waypoints()
