"""The path to the next global waypoint, represented by the `LocalPath` class."""

from custom_interfaces.msg import GPS, AISShips, GlobalPath, WindSensor
from rclpy.impl.rcutils_logger import RcutilsLogger

from local_pathfinding.ompl_path import OMPLPath


class LocalPathState:
    """Gathers and stores the state of the Sailbot.

    Attributes:
        postion (GPS): The latitude (float32) and longitudinal (float32) coordinates of the Sailbot.
        speed (GPS): The speed (float32) of the Sailbot at that position. Units: km/hr.
        heading (GPS): The direction (float32) in which the Sailbot is Sailing at. Units: Degrees.
        ais_ships (AISShips): Objects of all the ships (Ask Patrick) that surrounds the Sailbot.
        global_path (GlobalPath): Objects of all the global way points which the Sailbot will travel to.
        wind_speed (WindSensor): The wind speed. Units: km/hr (Check with Patrick).
        wind_direction (WindSensor): The wind direction towards the boat. Units: Degrees.
    """

    def __init__(
        self,
        gps: GPS,
        ais_ships: AISShips,
        global_path: GlobalPath,
        filtered_wind_sensor: WindSensor,
    ):
        if gps:  # TODO: remove when mocks can be run
            self.position = gps.lat_lon
            self.speed = gps.speed
            self.heading = gps.heading
            self.ais_ships = ais_ships
            self.global_path = global_path
            self.wind_speed = filtered_wind_sensor.speed
            self.wind_direction = filtered_wind_sensor.direction_degrees


class LocalPath:
    """The LocalPath sets and updates the local waypoints in between the

    Attributes:
        _logger (RcutilsLogger): The ROS logger of LocalPath.
        _ompl_path (OMPLPath): ompl path object.
        waypoints (List): Returns a list of waypoints coordinates (x,y)
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

        """
        state = LocalPathState(gps, ais_ships, global_path, filtered_wind_sensor)
        ompl_path = OMPLPath(parent_logger=self._logger, max_runtime=1.0, local_path_state=state)
        if ompl_path.solved:
            self._logger.info('Updating local path')
            self._update(ompl_path)

    def _update(self, ompl_path: OMPLPath):
        self._ompl_path = ompl_path
        self.waypoints = self._ompl_path.get_waypoints()

        print(type(self.waypoints))
