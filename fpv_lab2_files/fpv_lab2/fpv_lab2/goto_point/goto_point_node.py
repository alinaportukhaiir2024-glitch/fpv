import math
from geometry_msgs.msg import TwistStamped
from fpv_lab2.flight_test.flight_test_node import FlightTestNode
from fpv_lab2.goto_point.handlers import GOTO_HANDLERS

class GotoPointNode(FlightTestNode):
    def __init__(self):
        super().__init__()
        self._handlers = GOTO_HANDLERS

        self.TAKEOFF_ALTITUDE = 2.5

        self.declare_parameter("delta_x", 5.2)
        self.declare_parameter("delta_y", 0.2)
        self.declare_parameter("tolerance", 0.35)
        self.declare_parameter("max_speed", 0.6)
        self.declare_parameter("min_speed", 0.15)
        self.declare_parameter("slowdown_radius", 0.8)

        self.delta_x = self.get_parameter("delta_x").value
        self.delta_y = self.get_parameter("delta_y").value
        self.tolerance = self.get_parameter("tolerance").value
        self.max_speed = self.get_parameter("max_speed").value
        self.min_speed = self.get_parameter("min_speed").value
        self.slowdown_radius = self.get_parameter("slowdown_radius").value

        self.start_position = None
        self.waypoints = []
        self.current_wp_index = 0

        self.cmd_vel_publisher = self.create_publisher(TwistStamped, "/ap/v1/cmd_vel", 10)

    def odometry_callback(self, message):
        super().odometry_callback(message)
        if self.start_position is None and self.odometry is not None:
            x0 = message.pose.pose.position.x
            y0 = message.pose.pose.position.y
            self.start_position = (x0, y0)

            # Перевірені координати коридорів
            self.waypoints = [
                (x0 + 1.2, y0 + 0.0),  # WP1: поворот
                (x0 + 1.2, y0 + 2.0),  # WP2: верхній коридор
                (x0 + 4.5, y0 + 2.0),  # WP3: проліт уздовж коридору
                (x0 + self.delta_x, y0 + self.delta_y)  # WP4: фініш (5.2, 0.2)
            ]
            self.get_logger().info(f"Старт зафіксовано: ({x0:.2f}, {y0:.2f}) | Waypoints: {self.waypoints}")

    def current_position(self):
        if self.odometry is None:
            return None
        return (self.odometry.pose.pose.position.x, self.odometry.pose.pose.position.y)

    def get_current_target(self):
        if not self.waypoints:
            return None
        return self.waypoints[self.current_wp_index]

    def advance_waypoint(self):
        if self.current_wp_index < len(self.waypoints) - 1:
            self.current_wp_index += 1
            return True
        return False

    def stop_horizontal_motion(self):
        self.send_velocity(0.0, 0.0)

    def send_velocity(self, vx, vy):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        self.cmd_vel_publisher.publish(msg)
