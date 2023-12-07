"""The mock global path node. This node is responsible for sending the mock global path to
 Local Pathfinding"""

import csv
import os
import time

import numpy as np
import rclpy
from custom_interfaces.msg import GPS, HelperLatLon, Path
from pyproj import Geod
from rclpy.node import Node

ELLIPSOID = Geod(ellps="WGS84")
M_to_KM = 0.001

# Mock gps data to get things running until we have a running gps node
# TODO Remove when NET publishes GPS
MOCK_GPS = GPS(lat_lon=HelperLatLon(latitude=48.9594, longitude=-123.3634))

DEFAULT_PATH = "local_pathfinding/global_paths/mock_global_path.csv"
INTERVAL_SPACING = 30.0  # km
GLOBAL_PATH_UPDATE_DELAY = 5.0  # seconds


def main(args=None):
    rclpy.init(args=args)
    mock_global_path = MockGlobalPath()

    rclpy.spin(node=mock_global_path)

    mock_global_path.destroy_node()
    rclpy.shutdown()


class MockGlobalPath(Node):
    """Stores and sends the global path to the navigate node.

    Subscribers:
        gps_sub (Subscription): Subscribe to a `GPS` msg

    Publishers and their timers:
        global_path_pub (): Publishes a `Path` msg of the global path
        global_path_timer (Timer): The timer object used to periodically run the global path
        callback and check if the global path has updated.

    Attributes from subscribers:
        gps (GPS): Data from the GPS sensor.

    Attributes:
        global_path (Path): The global path that will be published for use by navigate.
        path_mod_tmstmp (Str): The modification timestamp of the csv file containing
        the global path

    Parameters:
        path_update_period (Double): The time interval for checking if the global path was changed
        global_path_filepath (Str): The filepath to the csv file containing the global path
        interval_spacing (Double): The distance between waypoints on the global path
    """

    def __init__(self):
        super().__init__(node_name="mock_global_path")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("path_update_period", rclpy.Parameter.Type.DOUBLE),
                ("global_path_filepath", rclpy.Parameter.Type.STRING),
                ("interval_spacing", rclpy.Parameter.Type.DOUBLE),
            ],
        )

        self.set_parameters(
            [
                rclpy.parameter.Parameter(
                    name="path_update_period",
                    type_=rclpy.Parameter.Type.DOUBLE,
                    value=GLOBAL_PATH_UPDATE_DELAY,
                ),
                rclpy.parameter.Parameter(
                    name="global_path_filepath",
                    type_=rclpy.Parameter.Type.STRING,
                    value=DEFAULT_PATH,
                ),
                rclpy.parameter.Parameter(
                    name="interval_spacing",
                    type_=rclpy.Parameter.Type.DOUBLE,
                    value=INTERVAL_SPACING,
                ),
            ]
        )

        # Subscribers
        self.gps_sub = self.create_subscription(
            msg_type=GPS, topic="gps", callback=self.gps_callback, qos_profile=10
        )

        # Publishers
        self.global_path_pub = self.create_publisher(
            msg_type=Path, topic="global_path", qos_profile=10
        )

        # Path update timer
        path_update_period = (
            self.get_parameter("path_update_period").get_parameter_value().double_value
        )
        self.get_logger().info(f"Got parameter: {path_update_period=}")

        self.global_path_timer = self.create_timer(
            timer_period_sec=path_update_period,
            callback=self.global_path_callback,
        )

        # Attributes
        self.gps = MOCK_GPS  # TODO Remove when NET publishes GPS
        self.global_path = None
        self.path_mod_tmstmp = None

    # Subscriber callbacks
    def gps_callback(self, msg: GPS):
        self.get_logger().info(f"Received data from {self.gps_sub.topic}: {msg}")
        self.gps = msg

    # Timer callbacks
    def global_path_callback(self):
        """Check if the global path csv file has changed, on a regular time interval.
        If it has changed, the new path is published.

        Global path can be changed by modifying mock_global_path.csv or setting the
        global_path_filepath parameter to a filepath to a new csv file.
        """

        file_path = self.get_parameter("global_path_filepath")._value

        # check when global path was changed last
        path_mod_tmstmp = time.ctime(os.path.getmtime(file_path))

        if path_mod_tmstmp != self.path_mod_tmstmp:
            # read in new path and publish
            global_path = Path()
            with open(file_path, "r") as file:
                reader = csv.reader(file)
                # skip header
                reader.__next__()
                for row in reader:
                    global_path.waypoints.append(
                        HelperLatLon(latitude=float(row[0]), longitude=float(row[1]))
                    )

            self.path_mod_tmstmp = path_mod_tmstmp

            if self.gps is not None:
                # check if global path is just a destination point
                if len(global_path.waypoints) < 2:
                    self.generate_path(dest=global_path.waypoints[0])
                else:
                    for i in range(1, len(global_path.waypoints)):
                        # check for duplicate waypoints
                        if (
                            global_path.waypoints[i].latitude
                            == global_path.waypoints[i - 1].latitude
                            and global_path.waypoints[i].longitude
                            == global_path.waypoints[i - 1].longitude
                        ):
                            self.get_logger().warn(
                                f"Found duplicate consecutive waypoint {global_path.waypoints[i]}"
                            )
                            # global_path.waypoints.pop(i)

                    self.global_path = global_path

                msg = self.global_path
                self.global_path_pub.publish(msg)
                self.get_logger().info(f"Publishing to {self.global_path_pub.topic}: {msg}")

            else:
                self.get_logger().warn("No GPS data")

    def generate_path(self, dest: HelperLatLon):
        """Generates and sets a path from the current GPS location to the destination point.
        Waypoints are evenly spaced along the path according to the interval_spacing parameter.
        """

        interval_spacing = self.get_parameter("interval_spacing")._value
        global_path = Path()

        pos = self.gps.lat_lon
        lat1 = pos.latitude
        lon1 = pos.longitude
        lat2 = dest.latitude
        lon2 = dest.longitude

        distance = ELLIPSOID.inv(lats1=lat1, lons1=lon1, lats2=lat2, lons2=lon2)[2] * M_to_KM
        n = np.ceil(distance / interval_spacing)

        global_path_tuples = ELLIPSOID.npts(
            lon1=lon1,
            lat1=lat1,
            lon2=lon2,
            lat2=lat2,
            npts=n,
        )

        for item in global_path_tuples:
            global_path.waypoints.append(
                # npts returns (lon,lat) tuples, its backwards for some reason
                HelperLatLon(
                    latitude=item[1],
                    longitude=item[0],
                )
            )

        # append the destination
        global_path.waypoints.append(
            HelperLatLon(
                latitude=lat2,
                longitude=lon2,
            )
        )

        self.global_path = global_path

    def _all_subs_active(self) -> bool:
        return self.gps is not None

    def _log_inactive_subs_warning(self):
        # TODO: log which subscribers are inactive
        self.get_logger().warning("Waiting for GPS to be published")


if __name__ == "__main__":
    main()
