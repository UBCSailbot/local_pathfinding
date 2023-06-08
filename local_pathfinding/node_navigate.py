import rclpy
from custom_interfaces.msg import AIS, GPS, GlobalPath, Heading, Wind
from rclpy.node import Node

from local_pathfinding.local_path import LocalPath


def main(args=None):
    rclpy.init(args=args)
    sailbot = Sailbot()

    rclpy.spin(node=sailbot)

    sailbot.destroy_node()
    rclpy.shutdown()


class Sailbot(Node):
    """Store, update, and maintain the state of our autonomous sailboat.

    Subscribers:
        ais_ships_sub (Subscription): Subscribe to a `AIS` msg.
        gps_sub (Subscription): Subscribe to a `GPS` msg.
        global_path_sub (Subscription): Subscribe to a `GlobalPath` msg.
        wind_sensors_sub (Subscription): Subscribe to a `Wind` msg.

    Publishers and their timers:
        desired_heading_pub (Publisher): Publish the desired heading in a `Heading` msg.
        desired_heading_timer (Timer): Call the desired heading callback function.

    Attributes from subscribers:
        ais_ships (AIS): Data from other boats.
        gps (GPS): Data from GPS sensor.
        global_path (GlobalPath): Path that we are following.
        wind_sensors (Wind): Data from wind sensors.

    Attributes:
        local_path (LocalPath): The path that `Sailbot` is following.
    """

    def __init__(self):
        super().__init__(node_name='navigate')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('pub_period_sec', rclpy.Parameter.Type.DOUBLE),
            ],
        )

        # subscribers
        self.ais_ships_sub = self.create_subscription(
            msg_type=AIS, topic='ais_ships', callback=self.ais_ships_call, qos_profile=10
        )
        self.gps_sub = self.create_subscription(
            msg_type=GPS, topic='gps', callback=self.gps_call, qos_profile=10
        )
        self.global_path_sub = self.create_subscription(
            msg_type=GlobalPath,
            topic='global_path',
            callback=self.global_path_call,
            qos_profile=10,
        )
        self.wind_sensors_sub = self.create_subscription(
            msg_type=Wind, topic='wind_sensors', callback=self.wind_sensors_call, qos_profile=10
        )

        # publishers and their timers
        self.desired_heading_pub = self.create_publisher(
            msg_type=Heading, topic='desired_heading', qos_profile=10
        )
        pub_period_sec = self.get_parameter('pub_period_sec').get_parameter_value().double_value
        self.get_logger().info(f'Got parameter: {pub_period_sec=}')
        self.desired_heading_timer = self.create_timer(
            timer_period_sec=pub_period_sec, callback=self.desired_heading_call
        )

        # attributes from subscribers
        self.ais_ships = None
        self.gps = None
        self.global_path = None
        self.wind_sensors = None

        # attributes
        self.local_path = LocalPath(parent_logger=self.get_logger())

    # subscriber callbacks

    def ais_ships_call(self, msg: AIS):
        self.get_logger().info(f'Received data from {self.ais_ships_sub.topic}: {msg}')
        self.ais_ships = msg

    def gps_call(self, msg: GPS):
        self.get_logger().info(f'Received data from {self.gps_sub.topic}: {msg}')
        self.gps = msg

    def global_path_call(self, msg: GlobalPath):
        self.get_logger().info(f'Received data from {self.global_path_sub.topic}: {msg}')
        self.global_path = msg

    def wind_sensors_call(self, msg: Wind):
        self.get_logger().info(f'Received data from {self.wind_sensors_sub.topic}: {msg}')
        self.wind_sensors = msg

    # publisher callbacks

    def desired_heading_call(self):
        """Get and publish the desired heading.

        Warn if not following the heading conventions in custom_interfaces/msg/Heading.msg.
        """
        desired_heading = self.get_desired_heading()
        if desired_heading < 0 or 360 <= desired_heading:
            self.get_logger().warning(f'Heading {desired_heading} not in [0, 360)')

        msg = Heading()
        msg.heading_degrees = desired_heading

        self.desired_heading_pub.publish(msg)
        self.get_logger().info(f'Publishing to {self.desired_heading_pub.topic}: {msg}')

    # get_desired_heading and its helper functions

    def get_desired_heading(self) -> float:
        """Get the desired heading.

        Returns:
            float: The desired heading if all subscribers are active, else a number that violates
                the heading convention.
        """
        if not self.all_subs_active():
            self.log_inactive_subs_warning()
            return -1.0

        self.local_path.update_if_needed()

        return 0.0

    def all_subs_active(self) -> bool:
        return True  # TODO: this line is a placeholder, delete when mocks can be run
        return self.ais_ships and self.gps and self.global_path and self.wind_sensors

    def log_inactive_subs_warning(self):
        # TODO: log which subscribers are inactive
        self.get_logger().warning('There are inactive subscribers')


if __name__ == '__main__':
    main()
