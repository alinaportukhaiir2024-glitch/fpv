import time
from functools import partial
from typing import TYPE_CHECKING
from ardupilot_msgs.srv import ArmMotors, Takeoff
from ..flight_state import FlightState
from .handler import FlightStateHandler

if TYPE_CHECKING:
    from ..flight_test_node import FlightTestNode

class ArmingHandler(FlightStateHandler):
    def __init__(self):
        self.last_arm_attempt = 0.0

    def handle(self, node: "FlightTestNode") -> None:
        now = time.monotonic()
        if node.request_in_progress or (now - self.last_arm_attempt < 1.5):
            return
        self.call_arm(node)

    def call_arm(self, node: "FlightTestNode") -> None:
        node.request_in_progress = True
        self.last_arm_attempt = time.monotonic()
        request = ArmMotors.Request()
        request.arm = True
        future = node.arm_client.call_async(request)
        future.add_done_callback(partial(self.arm_callback, node))
        node.get_logger().info("Requesting motor arming...")

    def arm_callback(self, node: "FlightTestNode", future) -> None:
        try:
            response = future.result()
        except Exception as error:
            node.request_in_progress = False
            node.get_logger().warning(f"Arming service failed: {error}. Retrying...")
            return

        if not response.result:
            node.request_in_progress = False
            node.get_logger().warn("ArduPilot rejected arming, retrying...")
            return

        node.get_logger().info("Motors armed successfully!")
        
        target_alt = getattr(node, 'TAKEOFF_ALTITUDE', 2.5)
        takeoff_req = Takeoff.Request()
        takeoff_req.alt = float(target_alt)
        
        takeoff_future = node.takeoff_client.call_async(takeoff_req)
        takeoff_future.add_done_callback(partial(self.takeoff_callback, node))
        node.get_logger().info(f"Requesting takeoff to {target_alt} m...")

    def takeoff_callback(self, node: "FlightTestNode", future) -> None:
        node.request_in_progress = False
        try:
            response = future.result()
            node.get_logger().info("Takeoff command accepted!")
        except Exception as error:
            node.get_logger().warning(f"Takeoff call failed: {error}")
            
        node.state = FlightState.TAKING_OFF
