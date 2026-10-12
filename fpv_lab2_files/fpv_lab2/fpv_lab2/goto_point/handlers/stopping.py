import time
from geometry_msgs.msg import TwistStamped
from ardupilot_msgs.srv import ModeSwitch
from fpv_lab2.flight_test.handlers.handler import FlightStateHandler

class StoppingHandler(FlightStateHandler):
    def __init__(self):
        self.landing_requested = False
        self.mode_switched = False

    def handle(self, node) -> None:
        if node.odometry is None:
            return

        current_z = node.odometry.pose.pose.position.z

        if not hasattr(node, 'hover_started_at') or node.hover_started_at is None:
            node.hover_started_at = time.monotonic()
            node.stop_horizontal_motion()
            node.get_logger().info("Дрон зафіксовано у фінальній точці. Початок утримання (30 сек)...")

        elapsed = time.monotonic() - node.hover_started_at

        # 1. Фаза утримання (30 секунд)
        if elapsed < 30.0:
            node.stop_horizontal_motion()
            node.get_logger().info(
                f"Утримання позиції... Залишилось: {30.0 - elapsed:.1f} с", 
                throttle_duration_sec=2.0
            )
            return

        # 2. Перемикання в режим LAND
        if not self.mode_switched:
            self.mode_switched = True
            node.get_logger().info("30 секунд утримання вичерпано. Переключення в режим LAND...")
            req = ModeSwitch.Request()
            req.mode = 9  # 9 = LAND mode for ArduCopter
            node.mode_client.call_async(req)

        # 3. Примусове плавне зниження Vz = -0.5 м/с до землі
        if current_z > 0.25:
            node.get_logger().info(
                f"Зниження та посадка... Поточна висота Z: {current_z:.2f} м", 
                throttle_duration_sec=1.5
            )
            msg = TwistStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.twist.linear.x = 0.0
            msg.twist.linear.y = 0.0
            msg.twist.linear.z = -0.5  # Плавне зниження
            node.cmd_vel_publisher.publish(msg)
        else:
            # Дрон торкнувся землі
            node.stop_horizontal_motion()
            node.get_logger().info("Дрон успішно торкнувся землі! Місію повністю завершено.")
            node.destroy_node()
            import sys
            sys.exit(0)
