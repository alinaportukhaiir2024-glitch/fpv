import time
from functools import partial
from typing import TYPE_CHECKING
from std_srvs.srv import Trigger
from ..flight_state import FlightState
from .handler import FlightStateHandler

if TYPE_CHECKING:
    from ..flight_test_node import FlightTestNode

class PrearmCheckHandler(FlightStateHandler):
    def __init__(self):
        self.last_attempt_time = 0.0

    def handle(self, node: "FlightTestNode") -> None:
        # Чекаємо 2 секунди між повторними спробами, щоб не спамити сервіс
        current_time = time.monotonic()
        if node.request_in_progress or (current_time - self.last_attempt_time < 2.0):
            return

        self.call_prearm_check(node)

    def call_prearm_check(self, node: "FlightTestNode") -> None:
        node.request_in_progress = True
        self.last_attempt_time = time.monotonic()
        
        request = Trigger.Request()
        future = node.prearm_client.call_async(request)
        future.add_done_callback(partial(self.prearm_callback, node))
        node.get_logger().info("Running pre-arm check...")

    def prearm_callback(self, node: "FlightTestNode", future) -> None:
        node.request_in_progress = False
        try:
            response = future.result()
        except Exception as error:
            node.get_logger().warning(f"Pre-arm call error: {error}")
            return

        if not response.success:
            node.get_logger().warn(f"Vehicle not ready yet ({response.message}). Waiting 2s for EKF...")
            return

        node.get_logger().info(f"Pre-arm check passed: {response.message}")
        node.state = FlightState.SETTING_GUIDED
