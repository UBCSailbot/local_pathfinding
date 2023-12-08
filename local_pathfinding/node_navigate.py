"""The main node of the local_pathfinding package, represented by the `Sailbot` class."""

import rclpy
from custom_interfaces.msg import GPS, AISShips, DesiredHeading, Path, WindSensor
from rclpy.node import Node

from local_pathfinding.local_path import LocalPath


def main(args=None):
    rclpy.init(args=args)
    sailbot = Sailbot()

    rclpy.spin(node=sailbot)

    sailbot.destroy_node()
    rclpy.shutdown()


class Sailbot(Node):
    """Stores, updates, and maintains the state of our autonomous sailboat.

    Subscribers:
        ais_ships_sub (Subscription): Subscribe to a `AISShips` msg.
        gps_sub (Subscription): Subscribe to a `GPS` msg.
        global_path_sub (Subscription): Subscribe to a `Path` msg.
        filtered_wind_sensor_sub (Subscription): Subscribe to a `WindSensor` msg.

    Publishers and their timers:
        desired_heading_pub (Publisher): Publish the desired heading in a `DesiredHeading` msg.
        desired_heading_timer (Timer): Call the desired heading callback function.

    Attributes from subscribers:
        ais_ships (AISShips): Data from other boats.
        gps (GPS): Data from the GPS sensor.
        global_path (Path): Path that we are following.
        filtered_wind_sensor (WindSensor): Filtered data from the wind sensors.

    Attributes:
        local_path (LocalPath): The path that `Sailbot` is following.
    """

    def __init__(self):
        super().__init__(node_name="navigate")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("pub_period_sec", rclpy.Parameter.Type.DOUBLE),
            ],
        )

        # subscribers
        self.ais_ships_sub = self.create_subscription(
            msg_type=AISShips, topic="ais_ships", callback=self.ais_ships_callback, qos_profile=10
        )
        self.gps_sub = self.create_subscription(
            msg_type=GPS, topic="gps", callback=self.gps_callback, qos_profile=10
        )
        self.global_path_sub = self.create_subscription(
            msg_type=Path, topic="global_path", callback=self.global_path_callback, qos_profile=10
        )
        self.filtered_wind_sensor_sub = self.create_subscription(
            msg_type=WindSensor,
            topic="filtered_wind_sensor",
            callback=self.filtered_wind_sensor_callback,
            qos_profile=10,
        )

        # publishers and their timers
        self.desired_heading_pub = self.create_publisher(
            msg_type=DesiredHeading, topic="desired_heading", qos_profile=10
        )
        pub_period_sec = self.get_parameter("pub_period_sec").get_parameter_value().double_value
        self.get_logger().debug(f"Got parameter: {pub_period_sec=}")
        self.desired_heading_timer = self.create_timer(
            timer_period_sec=pub_period_sec, callback=self.desired_heading_callback
        )

        # attributes from subscribers
        self.ais_ships = None
        self.gps = None
        self.global_path = None
        self.filtered_wind_sensor = None

        # attributes
        self.local_path = LocalPath(parent_logger=self.get_logger())

    # subscriber callbacks

    def ais_ships_callback(self, msg: AISShips):
        self.get_logger().debug(f"Received data from {self.ais_ships_sub.topic}: {msg}")
        self.ais_ships = msg

    def gps_callback(self, msg: GPS):
        self.get_logger().debug(f"Received data from {self.gps_sub.topic}: {msg}")
        self.gps = msg

    def global_path_callback(self, msg: Path):
        self.get_logger().debug(f"Received data from {self.global_path_sub.topic}: {msg}")
        self.global_path = msg

    def filtered_wind_sensor_callback(self, msg: WindSensor):
        self.get_logger().debug(f"Received data from {self.filtered_wind_sensor_sub.topic}: {msg}")
        self.filtered_wind_sensor = msg

    # publisher callbacks

    def desired_heading_callback(self):
        """Get and publish the desired heading.

        Warn if not following the heading conventions in custom_interfaces/msg/HelperHeading.msg.
        """
        desired_heading = self.get_desired_heading()
        if desired_heading < 0 or 360 <= desired_heading:
            self.get_logger().warning(f"Heading {desired_heading} not in [0, 360)")

        msg = DesiredHeading()
        msg.heading.heading = desired_heading

        self.desired_heading_pub.publish(msg)
        self.get_logger().debug(f"Publishing to {self.desired_heading_pub.topic}: {msg}")

    # get_desired_heading and its helper functions

    def get_desired_heading(self) -> float:
        """Get the desired heading.

        Returns:
            float: The desired heading if all subscribers are active, else a number that violates
                the heading convention.
        """
        if not self._all_subs_active():
            self._log_inactive_subs_warning()
            return -1.0

        self.local_path.update_if_needed(
            self.gps, self.ais_ships, self.global_path, self.filtered_wind_sensor
        )

        # TODO: create function to compute the heading from current position to next local waypoint
        return 0.0

    def _all_subs_active(self) -> bool:
        return True  # TODO: this line is a placeholder, delete when mocks can be run
        return self.ais_ships and self.gps and self.global_path and self.filtered_wind_sensor

    def _log_inactive_subs_warning(self):
        # TODO: log which subscribers are inactive
        self.get_logger().warning("There are inactive subscribers")


if __name__ == "__main__":
    main()
