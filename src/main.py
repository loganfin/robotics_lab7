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

        self.cart_ac = ActionClient(
            self, CartPose, f"/{namespace}/cartesian_pose"
        )

        self.conveyor_ac = ActionClient(
            self, Conveyor, f"/{namespace}/conveyor"
        )

        self.gripper_ac = ActionClient(
            self, SchunkGripper, f"/{namespace}/schunk_gripper"
        )

        self.proximity_topic = self.create_subscription(
            ProxReadings,
            f"/{namespace}/prox_readings",
            self.listener,
            qos_profile_sensor_data,
        )

    def listener(self, msg):
        self.current_prox_reading = msg.right

    def send_cart_action(self, index):
        self.cart_ac[index].wait_for_server()  # Wait till it's ready

        self.get_logger().info(f"p[{index}]")

        goal = CartPose.Goal()  # Make Goal

        goal.x = p[index][0]
        goal.y = p[index][1]
        goal.z = p[index][2]
        goal.w = p[index][3]
        goal.p = p[index][4]
        goal.r = p[index][5]

        result = self.cart_ac[index].send_goal(goal).result
        self.get_logger().info(f"Result {result}")

    def send_gripper_action(self, command):
        goal = SchunkGripper.Goal()
        goal.command = command

        result = self.gripper_ac.send_goal(goal).result
        self.get_logger().info(f"Result {result}")

    def send_conveyor_action(self, command):
        goal = Conveyor.Goal()
        goal.command = command

        result = self.conveyor_ac.send_goal(goal).result
        self.get_logger().info(f"Result {result}")

    def demo(self):
        # Reset
        self.send_conveyor_action("stop")
        self.send_gripper_action("open")
        self.send_cart_action(0)

        # Die table position
        self.send_cart_action(1)

        # Pick up die
        self.send_gripper_action("close")

        sleep(1)

        self.send_cart_action(0)

        # Start of conveyor (above)
        self.send_cart_action(2)

        # Start of conveyor (contact with belt)
        self.send_cart_action(3)

        # Release die
        self.send_gripper_action("open")
        sleep(1)

        # Start conveyor
        self.send_conveyor_action("forward")

        # Start of conveyor (above)
        self.send_cart_action(2)

        # End of conveyor (above)
        self.send_cart_action(4)

        # sleep(5)

        while self.current_prox_reading != True:
            pass

        # Wait for sensor to go low
        self.send_conveyor_action("stop")

        # End of conveyor (contact with belt)
        self.send_cart_action(5)

        # Pick up die
        self.send_gripper_action("close")
        sleep(1)

        # End of conveyor (above)
        self.send_cart_action(4)

        # These long sleeps are necessary because the server sometimes responds
        # before the robot finishes moving
        sleep(5)

        # Rest position
        self.send_cart_action(0)

        sleep(5)

        # Die table position + 90 degree roll rotation
        self.send_cart_action(6)

        # Release die
        self.send_gripper_action("open")
        sleep(1)

        # sleep(5)

        # Rest position
        self.send_cart_action(0)

        sleep(5)


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
