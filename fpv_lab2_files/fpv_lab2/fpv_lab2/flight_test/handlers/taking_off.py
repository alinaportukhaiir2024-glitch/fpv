from fpv_lab2.goto_point.goto_state import GotoState
from .handler import FlightStateHandler

class TakingOffHandler(FlightStateHandler):
    def handle(self, node) -> None:
        if node.odometry is None:
            return

        current_z = node.odometry.pose.pose.position.z
        target_alt = getattr(node, 'TAKEOFF_ALTITUDE', 2.5)

        node.get_logger().info(
            f"Зліт у процесі... Поточна висота Z: {current_z:.2f} м / Ціль: {target_alt:.1f} м",
            throttle_duration_sec=1.0
        )

        if current_z >= 1.5:
            node.get_logger().info("Висоту зльоту досягнуто! Перехід до виконання траєкторії...")
            node.state = GotoState.GOING_TO_TARGET
