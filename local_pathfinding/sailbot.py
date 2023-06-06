import rclpy
from custom_interfaces.msg import AIS, GPS, GlobalPath, Heading, Wind
from rclpy.node import Node


def main(args=None):
    rclpy.init(args=args)
    sailbot = Sailbot()

    rclpy.spin(node=sailbot)

    sailbot.destroy_node()
    rclpy.shutdown()


class Sailbot(Node):
    """
    Stores, updates, and maintains the state of our autonomous sailboat.

    Subscribers:
        ais_ships_sub (Subscription): Subscribes to a `AIS` msg.
        gps_sub (Subscription): Subscribes to a `GPS` msg.
        global_path_sub (Subscription): Subscribes to a `GlobalPath` msg.
        wind_sensors_sub (Subscription): Subscribes to a `Wind` msg.

    Data from subscribers:
        ais_ships (AIS): Data from other boats.
        gps (GPS): Data from GPS sensor.
        global_path (GlobalPath): Path that we are following.
        wind_sensors (Wind): Data from wind sensors.

    Publishers and their timers:
        desired_heading_pub (Publisher): Publishes the desired heading in a `Heading` msg.
        desired_heading_timer (Timer): Calls the desired heading callback function.
    """

    def __init__(self):
        super().__init__(node_name='sailbot')

        self.declare_parameters(
            namespace='', parameters=[('pub_period_sec', rclpy.Parameter.Type.DOUBLE)]
        )

        # subscribers
        self.ais_ships_sub = self.create_subscription(
            msg_type=AIS, topic='ais_ships', callback=self._ais_ships_call, qos_profile=10
        )
        self.gps_sub = self.create_subscription(
            msg_type=GPS, topic='gps', callback=self._gps_call, qos_profile=10
        )
        self.global_path_sub = self.create_subscription(
            msg_type=GlobalPath,
            topic='global_path',
            callback=self._global_path_call,
            qos_profile=10,
        )
        self.wind_sensors_sub = self.create_subscription(
            msg_type=Wind, topic='wind_sensors', callback=self._wind_sensors_call, qos_profile=10
        )

        # publishers
        self.desired_heading_pub = self.create_publisher(
            msg_type=Heading, topic='desired_heading', qos_profile=10
        )
        pub_period_sec = self.get_parameter('pub_period_sec').get_parameter_value().double_value
        self.get_logger().info(f'Got parameter: {pub_period_sec=}')
        self.desired_heading_timer = self.create_timer(
            timer_period_sec=pub_period_sec, callback=self._desired_heading_call
        )

    # subscriber callbacks

    def _ais_ships_call(self, msg: AIS):
        self.get_logger().info(f'Received data from {self.ais_ships_sub.topic}: {msg}')
        self.ais_ships = msg

    def _gps_call(self, msg: GPS):
        self.get_logger().info(f'Received data from {self.gps_sub.topic}: {msg}')
        self.gps = msg

    def _global_path_call(self, msg: GlobalPath):
        self.get_logger().info(f'Received data from {self.global_path_sub.topic}: {msg}')
        self.global_path = msg

    def _wind_sensors_call(self, msg: Wind):
        self.get_logger().info(f'Received data from {self.wind_sensors_sub.topic}: {msg}')
        self.wind_sensors = msg

    # publisher callbacks

    def _desired_heading_call(self):
        msg = Heading()
        msg.heading_degrees = 0.0

        self.desired_heading_pub.publish(msg)
        self.get_logger().info(f'Publishing to {self.desired_heading_pub.topic}: {msg}')


if __name__ == '__main__':
    main()
