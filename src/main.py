import sys
from time import sleep

# ROS packages
import rclpy
from rclpy.action.client import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

sys.path.append("../dependencies/")
# Fanuc packages
import fanuc_interfaces
from fanuc_interfaces.action import CartPose, Conveyor, SchunkGripper
from fanuc_interfaces.msg import ProxReadings
from key_commander import KeyCommander

# This is important for running your nodes from the terminal
from pynput.keyboard import KeyCode

namespace = "bunsen"

p = [
    # Rest position
    [522.7, 5.9, 96.4, 179.9, 0.0, 30.0],
    # Die table position
    [528.298, 3.742, -196.421, -179.9, 0.0, 30.0],
    # Start of conveyor (above)
    [895.0, -596.086243, -39.261475, 179.9, 0.0, 30.0],
    # Start of conveyor (contact with belt)
    [895.0, -593.533020, -205.173096, -179.9, 0.0, 30.0],
    # End of conveyor (above)
    [77.675018, -575.866882, -27.547470, 179.9, 0.0, 30.0],
    # End of conveyor (contact with belt)
    [70.0, -574.967712, -203.650116, -179.9, 0.0, 30.0],
    # Die table position + 90 degree roll rotation
    [528.298, 3.742, -196.421, -179.9, 0.0, 120.0],
]


class Demo(Node):
    def __init__(self, namespace):
        super().__init__("robot")

        self.actions = {
            "cartesian": ActionClient(
                self, CartPose, f"/{namespace}/cartesian_pose"
            ),
            "conveyor": ActionClient(self, Conveyor, f"/{namespace}/conveyor"),
            "gripper": ActionClient(
                self, SchunkGripper, f"/{namespace}/schunk_gripper"
            ),
        }

        # self.topics = {
        #     "proximity": self.create_subscription(
        #         ProxReadings,
        #         f"/{namespace}/prox_readings",
        #         self.listener,
        #         qos_profile_sensor_data,
        #     )
        # }

    def listener(self, msg):
        self.current_prox_reading = msg.right

    def send_cart_goal(self, position):
        goal = CartPose.Goal()  # Make Goal

        goal.x = position[0]
        goal.y = position[1]
        goal.z = position[2]
        goal.w = position[3]
        goal.p = position[4]
        goal.r = position[5]

        result = self.actions["cartesian"].send_goal(goal).result
        self.get_logger().info(f"Result {result}")

    def send_gripper_goal(self, command):
        goal = SchunkGripper.Goal()
        goal.command = command

        result = self.actions["gripper"].send_goal(goal).result
        self.get_logger().info(f"Result {result}")

    def send_conveyor_goal(self, command):
        goal = Conveyor.Goal()
        goal.command = command

        result = self.actions["conveyor"].send_goal(goal).result
        self.get_logger().info(f"Result {result}")

    def demo(self):
        # Reset
        self.actions["conveyor"].wait_for_server()
        self.send_conveyor_goal("stop")

        self.actions["gripper"].wait_for_server()
        self.send_gripper_goal("open")

        self.actions["cartesian"].wait_for_server()
        self.send_cart_goal(p[0])


if __name__ == "__main__":
    rclpy.init()

    bunsen = Demo(namespace)

    # This allows us to start the function once the node is spinning
    keycom = KeyCommander(
        [
            (KeyCode(char="s"), bunsen.demo),
        ]
    )
    print("S")

    rclpy.spin(bunsen)  # Start executing the node
    rclpy.shutdown()
