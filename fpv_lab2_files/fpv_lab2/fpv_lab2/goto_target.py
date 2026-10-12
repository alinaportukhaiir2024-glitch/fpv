#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math
import time

from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
from ardupilot_msgs.srv import ModeSwitch, Takeoff
from std_srvs.srv import Trigger

class DroneGotoTargetNode(Node):
    def __init__(self):
        super().__init__('drone_goto_target')

        # Індивідуальні координати (Варіант: X=5.2, Y=0.2)
        self.target_x = 5.2
        self.target_y = 0.2
        self.target_z = 2.5
        self.tolerance = 0.25

        # Поточні координати
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.odom_received = False

        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        qos_system = QoSProfile(depth=10)

        # Підписка на топіки одометрії
        self.create_subscription(Odometry, '/model/iris/odometry', self.odom_callback, qos_best_effort)
        self.create_subscription(Odometry, '/iris/odometry', self.odom_callback, qos_best_effort)
        self.create_subscription(Odometry, '/odometry', self.odom_callback, qos_best_effort)

        # Видавець команд швидкості
        self.pub_vel = self.create_publisher(TwistStamped, '/ap/cmd_vel', 10)

        # Сервіси ArduPilot
        self.cli_mode = self.create_client(ModeSwitch, '/ap/mode_switch')
        self.cli_arm = self.create_client(Trigger, '/ap/arm_motors')
        self.cli_takeoff = self.create_client(Takeoff, '/ap/takeoff')

        self.state = 'INIT'
        self.last_cmd_time = time.time()
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f'Вузол запущено! Кінцева ціль: X={self.target_x}, Y={self.target_y}')

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z
        if not self.odom_received:
            self.get_logger().info('>>> УСПІШНО ОТРИМАНО ДАНІ ПОЗИЦІЇ З ODOMETRY!')
            self.odom_received = True

    def call_trigger(self, client):
        if client.wait_for_service(timeout_sec=0.1):
            req = Trigger.Request()
            client.call_async(req)

    def call_mode(self, mode_code):
        if self.cli_mode.wait_for_service(timeout_sec=0.1):
            req = ModeSwitch.Request()
            req.mode = mode_code
            self.cli_mode.call_async(req)

    def call_takeoff(self, alt):
        if self.cli_takeoff.wait_for_service(timeout_sec=0.1):
            req = Takeoff.Request()
            req.alt = float(alt)
            self.cli_takeoff.call_async(req)

    def control_loop(self):
        if not self.odom_received:
            self.get_logger().info('Чекаємо на перші дані одометрії...', throttle_duration_sec=2.0)
            return

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        if self.state == 'INIT':
            self.get_logger().info('>>> ЕТАП 1: Надсилання команд GUIDED, Arm та Takeoff...')
            self.call_mode(4)  # GUIDED
            self.call_trigger(self.cli_arm)
            self.call_takeoff(self.target_z)
            self.state = 'TAKEOFF_CHECK'
            self.start_time = time.time()

        elif self.state == 'TAKEOFF_CHECK':
            # Допоміжне постійне надсилання команд протягом 3 секунд для розблокування
            if time.time() - self.start_time < 3.0:
                self.call_mode(4)
                self.call_trigger(self.cli_arm)
                self.call_takeoff(self.target_z)
                cmd.twist.linear.z = 1.0
                self.pub_vel.publish(cmd)
            else:
                self.state = 'TAKEOFF'

        elif self.state == 'TAKEOFF':
            if self.current_z < self.target_z - 0.2:
                cmd.twist.linear.z = 1.0
                self.pub_vel.publish(cmd)
                self.get_logger().info(f'Зліт... Поточна висота: {self.current_z:.2f} м', throttle_duration_sec=1.0)
            else:
                self.get_logger().info('>>> ЕТАП 2: Досягнуто заданої висоти. Початок руху до цілі.')
                self.state = 'NAVIGATE'

        elif self.state == 'NAVIGATE':
            dx = self.target_x - self.current_x
            dy = self.target_y - self.current_y
            dist = math.sqrt(dx**2 + dy**2)

            self.get_logger().info(
                f'Позиція: X={self.current_x:.2f}, Y={self.current_y:.2f} | Відстань до цілі: {dist:.2f} м',
                throttle_duration_sec=0.8
            )

            if dist > self.tolerance:
                kp = 0.8
                vx = kp * dx
                vy = kp * dy

                speed = math.sqrt(vx**2 + vy**2)
                if speed > 1.2:
                    vx = (vx / speed) * 1.2
                    vy = (vy / speed) * 1.2

                cmd.twist.linear.x = vx
                cmd.twist.linear.y = vy
                self.pub_vel.publish(cmd)
            else:
                cmd.twist.linear.x = 0.0
                cmd.twist.linear.y = 0.0
                self.pub_vel.publish(cmd)
                
                self.get_logger().info('==================================================')
                self.get_logger().info('>>> ЦІЛЬ ДОСЯГНУТО!')
                self.get_logger().info(f'Фактичні координати перед посадкою: X={self.current_x:.3f}, Y={self.current_y:.3f}')
                error = math.sqrt((self.target_x - self.current_x)**2 + (self.target_y - self.current_y)**2)
                self.get_logger().info(f'Розрахована похибка E = {error:.3f} м')
                self.get_logger().info('==================================================')

                self.state = 'LAND'

        elif self.state == 'LAND':
            self.get_logger().info('>>> ЕТАП 3: Автоматична посадка (LAND)')
            self.call_mode(9)  # LAND
            if self.current_z < 0.3:
                self.get_logger().info('>>> УСПІШНО ПРИЗЕМЛИВСЯ! Завершення роботи.')
                self.timer.cancel()
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = DroneGotoTargetNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
