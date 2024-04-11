import random
import sys
from time import sleep, time_ns

# ROS packages
import rclpy
from rclpy.action.client import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64

sys.path.append("../dependencies/")
# Fanuc packages
import fanuc_interfaces
from fanuc_interfaces.action import CartPose, Conveyor, SchunkGripper
from fanuc_interfaces.msg import ProxReadings
from key_commander import KeyCommander

# This is important for running your nodes from the terminal
from pynput.keyboard import KeyCode

namespace = "bunsen"

positions = {
    "start": [522.0, 5.0, -40.0, 179.9, 0.0, 30.0],
    "die_start": [522.0, 5.0, -195.0, -179.9, 0.0, 30.0],
    "conveyor_front_above": [749.0, -575.0, -40.0, 179.9, 0.0, 30.0],
    "conveyor_front_almost": [749.0, -575.0, -130.0, 179.9, 0.0, 30.0],
    "conveyor_front_belt": [749.0, -575.0, -203.0, -179.9, 0.0, 30.0],
    "conveyor_front_above_roll": [749.0, -575.0, -40.0, 179.9, 0.0, 120.0],
    "conveyor_front_almost_roll": [749.0, -575.0, -130.0, 179.9, 0.0, 120.0],
    "conveyor_front_belt_roll": [749.0, -575.0, -203.0, -179.9, 0.0, 120.0],
    "conveyor_back_above": [
        77.675018,
        -575.866882,
        -27.547470,
        179.9,
        0.0,
        30.0,
    ],
    "conveyor_back_belt": [70.0, -574.967712, -203.650116, -179.9, 0.0, 30.0],
}

offset = 322.77


def get_distance_traveled(time: float) -> float:
    # Given a time that the belt was on, return the difference traveled

    # Conveyor belt is about 4ft from beginning to end
    # From the conveyor's product page, the approximate max speed is 17 FPM
    # This seems pretty good for right now for 1 sec
    # return time * 112
    distance = 107.1 * time - 36.08

    print(f"Time: {time}, Distance: {distance}")
    # return 104.0 * time - b
    return distance


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

        self.publisher = self.create_publisher(
            Float64,
            "BenLoganBunsen",
            qos_profile_sensor_data,
        )

        # self.timer = self.create_timer(time, self.timer_callback)
        # self.timer.cancel()

    def create_subscriber(self):
        self.subscriber = self.create_subscription(
            Float64,
            "Ben/conveyor_time",
            self.beaker_callback,
            qos_profile_sensor_data,
        )

    def beaker_callback(self, msg):
        self.beaker_rx = True
        self.beaker_offset = get_distance_traveled(msg.data)
        self.get_logger().info(f"/Ben/conveyor_time: {msg.data}")
        pass

    # def timer_callback(self):
    #     self.send_conveyor_goal("stop")
    #     self.timer.cancel()

    #     self.get_logger().info("Timer")

    #     self.get_logger().info("Awaken")

    #     temp_pos_above = positions["conveyor_front_above"].copy()
    #     temp_pos_above[0] -= get_distance_traveled(time)

    #     temp_pos_belt = positions["conveyor_front_belt"].copy()
    #     temp_pos_belt[0] = temp_pos_above[0]

    #     print(temp_pos_above)

    #     self.send_cart_goal(temp_pos_above)
    #     self.send_cart_goal(temp_pos_belt)

    #     self.send_gripper_goal("close")
    #     sleep(1)

    #     self.send_cart_goal(temp_pos_above)

    #     self.send_cart_goal(positions["start"])

    #     self.send_cart_goal(positions["die_start"])

    #     self.send_gripper_goal("open")
    #     sleep(1)

    #     self.send_cart_goal(positions["start"])

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
        self.actions["conveyor"].wait_for_server()

        goal = Conveyor.Goal()
        goal.command = command

        result = self.actions["conveyor"].send_goal(goal).result
        self.get_logger().info(f"Result {result}")

    def grab(self):

        temp_pos_belt = positions["conveyor_front_belt"].copy()
        temp_pos_belt[0] -= offset

        self.send_cart_goal(temp_pos_belt)

        input()

        temp_pos_almost = positions["conveyor_front_almost"].copy()
        temp_pos_almost[0] -= offset

        self.send_cart_goal(temp_pos_almost)

    def move(self):
        self.actions["conveyor"].wait_for_server()

        temp_pos_almost = positions["conveyor_front_almost"].copy()
        temp_pos_almost[0] -= offset

        self.send_cart_goal(temp_pos_almost)

    def demo(self):
        self.reset()

        time = random.uniform(1, 5)
        # time = 3

        self.get_logger().info(f"Sleeping for {time} seconds")

        self.send_conveyor_goal("forward")
        start_time = time_ns()
        # self.timer.reset()
        sleep(time)
        self.send_conveyor_goal("stop")
        elapsed_time = (time_ns() - start_time) / 10**9

        self.get_logger().info(f"Actually slept for {elapsed_time} seconds")

        temp_pos_above = positions["conveyor_front_above"].copy()
        temp_pos_above[0] -= get_distance_traveled(elapsed_time)

        # self.send_cart_goal(temp_pos_above)

        temp_pos_almost = positions["conveyor_front_almost"].copy()
        temp_pos_almost[0] = temp_pos_above[0]

        self.send_cart_goal(temp_pos_almost)

        # 5 seconds delay
        # 15 - 16 inches
        # 35.8

        # 1 second delay
        # 31.25 - 32.5 inches
        # 35.8

        # self.send_conveyor_goal("forward")
        # sleep(time)
        # self.send_conveyor_goal("stop")

        # new_pos = positions["conveyor_front_above"].copy()
        # new_pos[0] -= get_distance_traveled(time)

        # self.send_cart_goal(new_pos)

    def lab(self):
        self.reset()

        # Repeat x 2:

        # Wait for notification from Beaker

        # Use the time given by Beaker's topic to calculate distance offset

        # Grab die of Beaker's belt

        # Rotate die about the y-axis

        # Move die to own belt

        # If prox sensor doesn't detect die, move to crash position

        # Start conveyor

        # Send time over own topic

        # Return to start position

        time = random.uniform(1, 5)
        self.send_conveyor_goal("forward")
        start_time = time_ns()
        # self.timer.reset()
        sleep(time)
        self.send_conveyor_goal("stop")
        elapsed_time = (time_ns() - start_time) / 10**9

        self.get_logger().info(f"Actually slept for {elapsed_time} seconds")

        publish_msg = Float64()
        publish_msg.data = positions["conveyor_front_above"][0] - get_distance_traveled(elapsed_time)

        self.get_logger().info(f"Publishing {publish_msg.data}")

        self.publisher.publish(publish_msg)

        self.send_cart_goal(positions["start"])

    def reset(self):
        self.get_logger().info("Resetting robot...")
        self.beaker_rx = False
        self.actions["conveyor"].wait_for_server()
        self.send_conveyor_goal("stop")

        self.actions["gripper"].wait_for_server()
        self.send_gripper_goal("open")

        self.actions["cartesian"].wait_for_server()
        self.send_cart_goal(positions["start"])


if __name__ == "__main__":
    rclpy.init()

    bunsen = Demo(namespace)

    # This allows us to start the function once the node is spinning
    keycom = KeyCommander(
        [
            (KeyCode(char="t"), bunsen.create_subscriber),
            (KeyCode(char="s"), bunsen.demo),
            (KeyCode(char="l"), bunsen.lab),
            (KeyCode(char="g"), bunsen.grab),
        ]
    )
    print("'t': subscribe to topic")
    print("'s': move die on conveyor")
    print("'l': send beaker message")
    print("'g': move to belt position")

    rclpy.spin(bunsen)  # Start executing the node
    rclpy.shutdown()
