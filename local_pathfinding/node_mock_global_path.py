"""The main node of the local_pathfinding package, represented by the `Sailbot` class."""

import csv
import os
import time
from typing import List

import numpy as np
import rclpy
from custom_interfaces.msg import GPS, HelperLatLon, Path
from custom_interfaces.srv import GlobalPath
from rclpy.node import Node

# Destination is hardcoded temporarily as a single element list
MOCK_DESTINATION = []
# List of doubles is an allowed type for a ros2 parameter
MOCK_DESTINATION.append("49.263,123.138")
# Mock gps data to get things running until we have a running gps node
MOCK_GPS = GPS(lat_lon=HelperLatLon(latitude=48.9594, longitude=-123.3634))

NUM_INTERVALS = 100
GLOBAL_PATH_UPDATE_DELAY = 10


def main(args=None):
    rclpy.init(args=args)
    mock_global_path = MockGlobalPath()

    rclpy.spin(node=mock_global_path)

    mock_global_path.destroy_node()
    rclpy.shutdown()


class MockGlobalPath(Node):
    """Stores and sends the global path to the navigate node.

    Subscribers:
        gps_sub (Subscription): Subscribe to a `GPS` msg.

    Services:
        global_path_srv (Service): Service to send the global path to node_navigate,
        as a request message.

    Attributes from subscribers:
        gps (GPS): Data from the GPS sensor.

    Attributes:
        global_path (List[Str]): The global path that will be converted to List[HelperLatLon] and
        sent to the navigate node as a service request message.
    """

    def __init__(self):
        super().__init__(node_name="mock_global_path")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("pub_period_sec", rclpy.Parameter.Type.DOUBLE),
                ("global_path_param", rclpy.Parameter.Type.STRING_ARRAY),
            ],
        )

        self.set_parameters(
            [
                rclpy.parameter.Parameter(
                    "global_path_param", rclpy.Parameter.Type.STRING_ARRAY, MOCK_DESTINATION
                )
            ]
        )
        # services
        self.client = self.create_client(GlobalPath, "global_path_srv")

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("connection to node_navigate failed, waiting again...")
        self.req = GlobalPath.Request()

        # subscribers
        self.gps_sub = self.create_subscription(
            msg_type=GPS, topic="gps", callback=self.gps_callback, qos_profile=10
        )

        pub_period_sec = self.get_parameter("pub_period_sec").get_parameter_value().double_value
        self.get_logger().info(f"Got parameter: {pub_period_sec=}")

        self.global_path_update_timer = self.create_timer(
            timer_period_sec=pub_period_sec * GLOBAL_PATH_UPDATE_DELAY,
            callback=self.global_path_callback,
        )

        # attributes
        self.gps = MOCK_GPS
        self.global_path = []
        self.path_mod_tmstmp = None

    # subscriber callbacks
    def gps_callback(self, msg: GPS):
        self.get_logger().info(f"Received data from {self.gps_sub.topic}: {msg}")
        self.gps = msg

    # Service callbacks
    def global_path_callback(self):
        """Check if the global path parameter has changed, on a regular time interval.
        If it has changed, the new path is sent to node_navigate

        Global path can be changed through a ros2 param set command in the terminal
        or by modifying mock_global_path.csv
        """

        # check when global path was changed last
        path_mod_tmstmp = time.ctime(
            os.path.getmtime("/local_pathfinding/resource/mock_global_path.csv")
        )

        if path_mod_tmstmp != self.path_mod_tmstmp:
            with open("resource/mock_global_path.csv") as file:
                reader = csv.reader(file)
                reader.next()
                global_path = []
                for row in reader:
                    global_path.append(row[0] + "," + row[1])

            self.path_mod_tmstmp = path_mod_tmstmp

            # Set the new global path parameter
            self.set_parameters(
                [
                    rclpy.parameter.Parameter(
                        "global_path_param",
                        rclpy.Parameter.Type.STRING_ARRAY,
                        global_path,
                    )
                ]
            )

        if self.global_path != self.get_parameter("global_path_param")._value:
            if self.gps is not None:
                self.set_global_path()
                self.send_global_path()
            else:
                self.get_logger().info("No GPS data")

        # check if navigate responded to the service call
        if self.future is not None and self.future.done():
            try:
                response = self.future.result()
            except Exception as e:
                self.get_logger().info("Failed to send global path %r" % (e,))
            else:
                self.get_logger().info(f"Navigate node response: {response.response}")
            self.future = None

    def set_global_path(self):
        """update the global path from the global path parameter value. If the global_path_param
        is a single LatLon point, then this function will create a path of evenly-ish spaced
        intermediate points"""

        if not self._all_subs_active():
            self._log_inactive_subs_warning()
            return

        global_path = self.get_parameter("global_path_param")._value

        # Check if global path parameter is just a destination point
        if len(global_path) < 2:
            # Create a simple global path location and destination
            current_location = self.gps.lat_lon

            latitudes = np.linspace(
                current_location.latitude, float(global_path[0].split(",")[0]), NUM_INTERVALS + 1
            )
            longitudes = np.linspace(
                current_location.longitude, float(global_path[0].split(",")[1]), NUM_INTERVALS + 1
            )

            global_path = []

            for i in range(NUM_INTERVALS):
                global_path.append(str(latitudes[i]) + "," + str(longitudes[i]))

            self.global_path = global_path

            # Set the new global path parameter
            self.set_parameters(
                [
                    rclpy.parameter.Parameter(
                        "global_path_param",
                        rclpy.Parameter.Type.STRING_ARRAY,
                        self.global_path,
                    )
                ]
            )

            return

        self.global_path = global_path

    def send_global_path(self):
        self.req.global_path = MockGlobalPath.str_list_to_path(self.global_path)
        self.future = self.client.call_async(self.req)

    def _all_subs_active(self) -> bool:
        return True  # TODO: this line is a placeholder, delete when mocks can be run

    def _log_inactive_subs_warning(self):
        # TODO: log which subscribers are inactive
        self.get_logger().warning("There are inactive subscribers")

    def str_list_to_path(global_path: List[str]) -> Path:
        """Convert a String array compatible with ROS2 parameter typing to a Path object"""
        HelperLatLon_array = []
        for item in global_path:
            HelperLatLon_array.append(
                HelperLatLon(
                    latitude=float(item.split(",")[0]),
                    longitude=float(item.split(",")[1]),
                )
            )
        return Path(waypoints=HelperLatLon_array)


if __name__ == "__main__":
    main()
