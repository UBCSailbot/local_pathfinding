"""The main node of the local_pathfinding package, represented by the `Sailbot` class."""

from typing import List

import rclpy
import numpy as np
from custom_interfaces.msg import GPS, HelperLatLon, Path
from rclpy.node import Node

# Destination is hardcoded temporarily as a single element list
DESTINATION = []
DESTINATION.append(HelperLatLon(lat=49.263, lon=-123.138))

NUM_INTERVALS = 100


def main(args=None):
    rclpy.init(args=args)
    global_path = GlobalPath()

    rclpy.spin(node=global_path)

    global_path.destroy_node()
    rclpy.shutdown()


class GlobalPath(Node):
    """Stores, and publishes the global path.

    Subscribers:
        gps_sub (Subscription): Subscribe to a `GPS` msg.

    Publishers and their timers:
        global_path_pub (Subscription): Subscribe to a `Path` msg.
        global_path_timer (Timer): Call the global path callback function.

    Attributes from subscribers:
        gps (GPS): Data from the GPS sensor.
    """

    def __init__(self):
        super().__init__(node_name="mock_global_path")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("pub_period_sec", rclpy.Parameter.Type.DOUBLE),
                ("global_path_param", DESTINATION),
            ],
        )

        # subscribers
        self.gps_sub = self.create_subscription(
            msg_type=GPS, topic="gps", callback=self.gps_callback, qos_profile=10
        )

        # publishers and their timers
        self.global_path_pub = self.create_publisher(
            msg_type=Path, topic="global_path", qos_profile=10
        )
        pub_period_sec = self.get_parameter("pub_period_sec").get_parameter_value().double_value
        self.get_logger().info(f"Got parameter: {pub_period_sec=}")
        self.global_path_timer = self.create_timer(
            timer_period_sec=pub_period_sec, callback=self.global_path_callback
        )

        # attributes from subscribers
        self.gps = None

    # subscriber callbacks

    def gps_callback(self, msg: GPS):
        self.get_logger().info(f"Received data from {self.gps_sub.topic}: {msg}")
        self.gps = msg

    # publisher callbacks

    def global_path_callback(self):
        """Get and publish the global path."""
        if self.gps is not None:
            global_path = self.get_global_path()

            if global_path is None or len(global_path) < NUM_INTERVALS + 1:
                self.get_logger().warning("Global path is invalid")

            msg = Path()
            msg.waypoints = global_path

            self.global_path_pub.publish(msg)
            self.get_logger().info(f"Publishing to {self.global_path_pub.topic}: {msg}")

    # get_global_path and its helper functions

    def get_global_path(self) -> List[HelperLatLon]:
        """Get the global path.

        Returns:
            List[HelperLatLon]: The global path.
        """
        if not self._all_subs_active():
            self._log_inactive_subs_warning()
            return []

        current_location = self.gps.latlon

        global_path = self.get_parameter("global_path_param").get_parameter_value

        # Check if global path parameter is just a destination point
        if len(global_path)<2:

            if len(global_path)<1:
                self.get_logger().warning("Global path has 0 elements. Must have at least 1.")
                # TODO do something here to handle this

            # Create a simple global path from the single destination point
            # There will be some distortion here, so the intermediate points wont be evenly spaced
            latitudes = np.linspace(current_location.lat, global_path[0].lat, NUM_INTERVALS+1)
            longitudes = np.linspace(current_location.lon, global_path[0].lon, NUM_INTERVALS+1)

            global_path = []

            for i in range(NUM_INTERVALS):
                global_path.append(
                    HelperLatLon(
                        lat=latitudes[i],
                        lon=longitudes[i],
                    )
                )
            return global_path

        return global_path

    def _all_subs_active(self) -> bool:
        return True  # TODO: this line is a placeholder, delete when mocks can be run

    def _log_inactive_subs_warning(self):
        # TODO: log which subscribers are inactive
        self.get_logger().warning("There are inactive subscribers")


if __name__ == "__main__":
    main()
