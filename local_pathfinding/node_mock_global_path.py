"""The mock global path node. This node is responsible for sending the mock global path to
 Local Pathfinding"""

import csv
import os
import time
from datetime import datetime

import numpy as np
import rclpy
from custom_interfaces.msg import GPS, HelperLatLon, Path
from rclpy.node import Node

from local_pathfinding.coord_systems import GEODESIC, meters_to_km

# TODO if csv already contains a path, but first waypoint is greater than interval_spacing away,
#  interpolate a path between GPS and the first waypoint and append it to the beginning of the path
#  Do this by running generate_path between GPS and waypoint[0]

# Mock gps data to get things running until we have a running gps node
# TODO Remove when NET publishes GPS
MOCK_GPS = GPS(lat_lon=HelperLatLon(latitude=49.1154488073483, longitude=-125.95696431913618))


def main(args=None):
    rclpy.init(args=args)
    mock_global_path = MockGlobalPath()

    rclpy.spin(node=mock_global_path)

    mock_global_path.destroy_node()
    rclpy.shutdown()


class MockGlobalPath(Node):
    """Stores and publishes the mock global path to the global_path topic.

    Subscribers:
        gps_sub (Subscription): Subscribe to a `GPS` msg

    Publishers and their timers:
        global_path_pub (): Publishes a `Path` msg containing the global path
        global_path_timer (Timer): The timer object used to periodically run the global path
        callback and check if the global path has been updated.

    Attributes from subscribers:
        gps (GPS): Data from the GPS sensor.

    Attributes:
        path_mod_tmstmp (Str): The modification timestamp of the csv file containing
        the global path
        file_path (Str): The filepath to the csv file containing the global path

    Parameters:
        pub_period_sec (Double): This parameter is used to set the period of the global path
        callback function
        global_path_filepath (Str): The filepath to the csv file containing the global path
        interval_spacing (Double): The distance between waypoints on the global path
    """

    def __init__(self):
        super().__init__(node_name="mock_global_path")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("pub_period_sec", rclpy.Parameter.Type.DOUBLE),
                ("global_path_filepath", rclpy.Parameter.Type.STRING),
                ("interval_spacing", rclpy.Parameter.Type.DOUBLE),
                ("write", rclpy.Parameter.Type.BOOL),
            ],
        )

        # Subscribers
        self.gps_sub = self.create_subscription(
            msg_type=GPS, topic="gps", callback=self.gps_callback, qos_profile=10
        )

        # Publishers
        self.global_path_pub = self.create_publisher(
            msg_type=Path, topic="global_path", qos_profile=10
        )

        # Path callback timer
        pub_period_sec = self.get_parameter("pub_period_sec").get_parameter_value().double_value
        self.get_logger().debug(f"Got parameter: {pub_period_sec=}")

        self.global_path_timer = self.create_timer(
            timer_period_sec=pub_period_sec,
            callback=self.global_path_callback,
        )

        # Attributes
        self.gps = MOCK_GPS  # TODO Remove when NET publishes GPS
        self.path_mod_tmstmp = None
        self.file_path = None

    # Subscriber callbacks
    def gps_callback(self, msg: GPS):
        self.get_logger().debug(f"Received data from {self.gps_sub.topic}: {msg}")
        self.gps = msg

    # Timer callbacks
    def global_path_callback(self):
        """Check if the global path csv file has changed, on a regular time interval.
        If it has changed, the new path is published.

        Depending on the boolean value of the write parameter, each generated path may be written
        to a new csv file in the same directory as the source csv file.

        Global path can be changed by modifying mock_global_path.csv or setting the
        global_path_filepath parameter to a filepath to a new csv file.
        """
        if not self._all_subs_active():
            self._log_inactive_subs_warning()

        file_path = self.get_parameter("global_path_filepath")._value

        # check when global path was changed last
        path_mod_tmstmp = time.ctime(os.path.getmtime(file_path))

        # TODO also check if GPS has changed more than some threshold amount
        # We want a new global path in this case too

        # Only publish if the path
        if (self.file_path != file_path) or (path_mod_tmstmp != self.path_mod_tmstmp):
            self.get_logger().info(
                f"Global path file changed to: {os.path.basename(file_path)}\n reading new path"
            )

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
            self.file_path = file_path

            # check if global path is just a destination point
            if len(global_path.waypoints) < 2:
                interval_spacing = self.get_parameter("interval_spacing")._value
                pos = self.gps.lat_lon
                write = self.get_parameter("write")._value

                self.get_logger().info(
                    f"Generating new path from {pos.latitude +','+ pos.longitude} to "
                    f"{global_path.waypoints[0].latitude +','+ global_path.waypoints[0].longitude}"
                )

                if write:
                    self.get_logger().info("Writing generated path to new file")

                msg = MockGlobalPath.generate_path(
                    dest=global_path.waypoints[0],
                    interval_spacing=interval_spacing,
                    pos=pos,
                    write=write,
                    file_path=file_path,
                )
            else:
                msg = global_path

            # publish global path
            self.global_path_pub.publish(msg)
            self.get_logger().info(
                f"Publishing to {self.global_path_pub.topic}: {MockGlobalPath.path_to_dict(msg)}"
            )

    @staticmethod
    def generate_path(
        dest: HelperLatLon,
        interval_spacing: float,
        pos: HelperLatLon,
        write: bool = False,
        file_path: str = "",
    ) -> Path:
        """Returns a path from the current GPS location to the destination point.
        Waypoints are evenly spaced along the path according to the interval_spacing parameter.
        Path does not include pos, but does include dest as the final element.

        If write is True, the path is written to a new csv file in the same directory as file_path,
        with the name of the original file, appended with a timestamp.

        Args:
            dest (HelperLatLon): The destination point
            interval_spacing (float): The desired distance between waypoints on the path
            pos (HelperLatLon): The current GPS location
            write (bool, optional): Whether to write the path to a new csv file, default False
            file_path (str, optional): The filepath to the csv file containing the global path,

        Returns:
            Path: The generated path
        """
        global_path = Path()

        lat1 = pos.latitude
        lon1 = pos.longitude
        lat2 = dest.latitude
        lon2 = dest.longitude

        distance = meters_to_km(GEODESIC.inv(lats1=lat1, lons1=lon1, lats2=lat2, lons2=lon2)[2])
        n = np.ceil(distance / interval_spacing)

        global_path_tuples = GEODESIC.npts(lon1=lon1, lat1=lat1, lon2=lon2, lat2=lat2, npts=n)

        # npts returns (lon,lat) tuples, its backwards for some reason
        for lon, lat in global_path_tuples:
            global_path.waypoints.append(HelperLatLon(latitude=lat, longitude=lon))

        # append the destination
        global_path.waypoints.append(HelperLatLon(latitude=lat2, longitude=lon2))

        if write:
            if file_path == "":
                raise ValueError("file_path must be specified when write is True")

            # write to a new timestamped csv file
            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

            dst_file_path = file_path.removesuffix(".csv") + f"_{timestamp}.csv"
            with open(dst_file_path, "w") as file:
                writer = csv.writer(file)
                writer.writerow(["latitude", "longitude"])
                for waypoint in global_path.waypoints:
                    writer.writerow([waypoint.latitude, waypoint.longitude])

        return global_path

    @staticmethod
    def path_to_dict(path: Path, num_decimals: int = 4) -> dict[int, str]:
        """Converts a Path msg to a dictionary suitable for printing.

        Args:
            path (Path): The Path msg to be converted.
            num_decimals (int, optional): The number of decimal places to round to, default 4.

        Returns:
            dict[int, str]: Keys are the indices of the formatted latlon waypoints.
        """
        return {
            i: f"({waypoint.latitude:.{num_decimals}f}, {waypoint.longitude:.{num_decimals}f})"
            for i, waypoint in enumerate(path.waypoints)
        }

    def _all_subs_active(self) -> bool:
        return self.gps is not None

    def _log_inactive_subs_warning(self):
        self.get_logger().warning("Waiting for GPS to be published")


if __name__ == "__main__":
    main()
