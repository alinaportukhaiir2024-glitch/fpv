#!/usr/bin/env python3
import rclpy
from fpv_lab2.flight_test.flight_state import FlightState
from fpv_lab2.goto_point.goto_point_node import GotoPointNode

def main(args=None):
    rclpy.init(args=args)
    node = GotoPointNode()
    try:
        while (
            rclpy.ok()
            and node.state != FlightState.FINISHED
            and node.state != FlightState.ERROR
        ):
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
