"""The mock global path node. This node is responsible for sending the mock global path to
 Local Pathfinding"""

import csv
import os
import time
from typing import List

import numpy as np
import rclpy
from custom_interfaces.msg import GPS, HelperLatLon, Path
from custom_interfaces.srv import GlobalPath
from pyproj import Geod
from rclpy.node import Node

GEODESIC = Geod(ellps="WGS84")
M_to_KM = 0.001

# Mock gps data to get things running until we have a running gps node
# TODO Remove when NET publishes GPS
MOCK_GPS = GPS(lat_lon=HelperLatLon(latitude=48.9594, longitude=-123.3634))
DEFAULT_PATH = "global_paths/mock_global_path.csv"

INTERVAL_SPACING = 30  # km
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

        path_mod_tmstmp (Str): The timestamp of the last time the source global path csv was
        modified.

        future (Future): The future object returned by the service call to node_navigate.

        client (Client): The client object used to send the global path to node_navigate.

        global_path_update_timer (Timer): The timer object used to periodically run the global path
        callback.

        req (Request): The request object used to send the global path to node_navigate.
    """

    def __init__(self):
        super().__init__(node_name="mock_global_path")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("pub_period_sec", rclpy.Parameter.Type.DOUBLE),
                ("global_path_filepath", rclpy.Parameter.Type.STRING),
                ("interval_spacing", rclpy.Parameter.Type.DOUBLE),
            ],
        )

        self.set_parameters(
            [
                rclpy.parameter.Parameter(
                    name="global_path_filepath",
                    type_=rclpy.Parameter.Type.STRING_ARRAY,
                    value=DEFAULT_PATH,
                ),
                rclpy.parameter.Parameter(
                    name="interval_spacing",
                    type_=rclpy.Parameter.Type.DOUBLE,
                    value=INTERVAL_SPACING,
                ),
            ]
        )
        # services
        self.client = self.create_client(GlobalPath, "global_path_srv")

        # wait for connection to node_navigate
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("connection to node_navigate failed, waiting again...")
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
        self.gps = MOCK_GPS  # TODO Remove when NET publishes GPS
        self.global_path = []
        self.path_mod_tmstmp = None
        self.future = None

    # subscriber callbacks
    def gps_callback(self, msg: GPS):
        self.get_logger().info(f"Received data from {self.gps_sub.topic}: {msg}")
        self.gps = msg

    # Service callbacks
    def global_path_callback(self):
        """Check if the global path csv file has changed, on a regular time interval.
        If it has changed, the new path is sent to node_navigate

        Global path can be changed by modifying mock_global_path.csv or setting the
        global_path_filepath parameter to a filepath to a new csv file.
        """

        file_path = self.get_parameter("global_path_filepath")._value

        # check when global path was changed last
        path_mod_tmstmp = time.ctime(os.path.getmtime(file_path))

        if path_mod_tmstmp != self.path_mod_tmstmp:
            # read in new path and send to node_navigate
            global_path = []
            with open(file_path, "r") as file:
                reader = csv.reader(file)
                # skip header
                reader.__next__()
                for row in reader:
                    global_path.append(row[0] + "," + row[1])

            self.path_mod_tmstmp = path_mod_tmstmp

            if self.gps is not None:
                if len(global_path) < 2:
                    self.generate_path(global_path)
                else:
                    self.global_path = global_path

                self.send_global_path()
            else:
                self.get_logger().warn("No GPS data")

        # check if navigate responded to the service call
        if self.future is not None and self.future.done():
            try:
                response = self.future.result()
            except Exception as e:
                self.get_logger().warn("Failed to send global path %r" % (e,))
            else:
                self.get_logger().info(f"Navigate node response: {response.response}")
            self.future = None

    def generate_path(self, destination: List[str]):
        """Generates a path from the current GPS location to the destination point. Waypoints are
        evenly spaced along the path according to the interval_spacing parameter."""

        interval_spacing = self.get_parameter("interval_spacing")._value
        global_path = []

        pos = self.gps.lat_lon
        dest = HelperLatLon(
            latitude=float(destination[0].split("'")[0]),
            longitude=float(destination[0].split(",")[1]),
        )

        distance = (
            GEODESIC.inv(pos.longitude, pos.latitude, dest.longitude, dest.latitude)[2] * M_to_KM
        )

        n = np.ceil(distance / interval_spacing)

        global_path = GEODESIC.npts(
            lon1=pos.longitude,
            lat1=pos.latitude,
            lon2=dest.longitude,
            lat2=dest.latitude,
            npts=n,
        )

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
