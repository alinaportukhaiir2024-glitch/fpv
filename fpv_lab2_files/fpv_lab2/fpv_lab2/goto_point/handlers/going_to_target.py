import math
import time
from fpv_lab2.flight_test.handlers.handler import FlightStateHandler
from fpv_lab2.goto_point.goto_state import GotoState

def get_yaw_from_quaternion(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class GoingToTargetHandler(FlightStateHandler):
    def __init__(self):
        self.stop_time = None

    def handle(self, node):
        position = node.current_position()
        if position is None or node.odometry is None:
            return

        target = node.get_current_target()
        dx_global = target[0] - position[0]
        dy_global = target[1] - position[1]
        distance = math.hypot(dx_global, dy_global)

        if distance <= node.tolerance:
            if self.stop_time is None:
                self.stop_time = time.monotonic()
                node.stop_horizontal_motion()
                node.get_logger().info(f">>> [WP {node.current_wp_index + 1} REACHED] Зупинка для стабілізації...")
                return

            if time.monotonic() - self.stop_time < 0.8:
                node.stop_horizontal_motion()
                return

            self.stop_time = None
            if node.advance_waypoint():
                node.get_logger().info(f">>> [NEXT WP] Перехід до WP [{node.current_wp_index + 1}/{len(node.waypoints)}]: {node.get_current_target()}")
                return
            else:
                node.get_logger().info(f"Фінальну ціль досягнуто! Відстань: {distance:.3f} м. Зупинка...")
                node.stop_horizontal_motion()
                node.state = GotoState.STOPPING
                return

        speed = node.max_speed * min(1.0, distance / node.slowdown_radius)
        speed = max(speed, node.min_speed)

        vx_global = speed * (dx_global / distance)
        vy_global = speed * (dy_global / distance)

        q = node.odometry.pose.pose.orientation
        yaw = get_yaw_from_quaternion(q)

        vx_body =  vx_global * math.cos(yaw) + vy_global * math.sin(yaw)
        vy_body = -( -vx_global * math.sin(yaw) + vy_global * math.cos(yaw) )

        node.get_logger().info(
            f"WP [{node.current_wp_index + 1}/{len(node.waypoints)}]: Pos=({position[0]:.2f}, {position[1]:.2f}) | До WP: {distance:.2f} м | V_body=({vx_body:.2f}, {vy_body:.2f})",
            throttle_duration_sec=0.8
        )
        node.send_velocity(vx_body, vy_body)
