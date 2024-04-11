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
    "error": [600.0, -300.0, 500.0, 90.0, 30.0, 0.0],
    "start": [522.0, 5.0, -40.0, 179.9, 0.0, 30.0],
    "die_start": [522.0, 5.0, -195.0, -179.9, 0.0, 30.0],
    "conveyor_front_above": [749.0, -578.0, -40.0, 179.9, 0.0, 30.0],
    "conveyor_front_belt": [749.0, -578.0, -203.0, -179.9, 0.0, 30.0],
    "conveyor_front_above_roll": [749.0, -578.0, -40.0, 179.9, 0.0, 120.0],
    "conveyor_front_belt_roll": [749.0, -578.0, -203.0, -179.9, 0.0, 120.0],
    "conveyor_a_above": [0.0, -833.0, -40.0, -179.9, 0.0, 30.0],
    "conveyor_a_belt": [0.0, -833.0, -203.0, -179.9, 0.0, 30.0],
    "rotation_station_above": [410.0, 250.0, -80.0, 179.9, 0.0, 30.0],
    "rotation_station_above_roll": [410.0, 250.0, -80.0, 179.9, 0.0, 120.0],
    "rotation_station_table_roll": [410.0, 250.0, -195.0, 179.9, 0.0, 120.0],
    "rotation_station_side": [415.0, 350.0, -217.0, -90.0, 60.0, 179.9],
    "rotation_station_push": [415.0, 205.0, -217.0, -90.0, 60.0, 179.9],
}


def get_distance_traveled(time: float) -> float:
    return 107.1 * time - 36.08


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

        self.publisher = self.create_publisher(
            Float64,
            "BenLoganBunsen",
            qos_profile_sensor_data,
        )

        self.subscriber = self.create_subscription(
            Float64,
            "/BenLoganBeaker",
            self.beaker_callback,
            qos_profile_sensor_data,
        )

        self.prox_sensor = self.create_subscription(
            ProxReadings,
            f"/{namespace}/prox_readings",
            self.prox_sensor_callback,
            qos_profile_sensor_data,
        )

    def beaker_callback(self, msg):
        self.beaker_rx = True

        # Is there an offset that needs to be calculated
        self.beaker_x_coord = msg.data - 55
        self.get_logger().info(f"Beaker: {msg.data}")

    def prox_sensor_callback(self, msg):
        self.current_prox_reading = msg.left

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
        sleep(0.5)

    def send_conveyor_goal(self, command):
        self.actions["conveyor"].wait_for_server()

        goal = Conveyor.Goal()
        goal.command = command

        result = self.actions["conveyor"].send_goal(goal).result
        self.get_logger().info(f"Result {result}")

    def lab(self):
        self.reset()

        # Repeat x 2:
        for _ in range(2):
            self.get_logger().info("Waiting on Beaker...")

            # Wait for notification from Beaker
            while self.beaker_rx is not True:
                pass

            self.get_logger().info("Received Message...")
            self.beaker_rx = False

            # Message received, go to Die position
            die_pos_above = positions["conveyor_a_above"].copy()

            die_pos_above[0] = self.beaker_x_coord

            die_pos_belt = positions["conveyor_a_belt"].copy()

            die_pos_belt[0] = die_pos_above[0]

            self.send_cart_goal(die_pos_above)
            self.send_cart_goal(die_pos_belt)

            # Grab die off Beaker's belt
            self.send_gripper_goal("close")

            self.send_cart_goal(die_pos_above)

            # Rotate die about the y-axis
            self.send_cart_goal(positions["rotation_station_above"])
            self.send_cart_goal(positions["rotation_station_table_roll"])
            self.send_gripper_goal("open")
            self.send_cart_goal(positions["rotation_station_above_roll"])
            self.send_cart_goal(positions["rotation_station_side"])
            self.send_cart_goal(positions["rotation_station_push"])
            self.send_gripper_goal("close")
            self.send_cart_goal(positions["rotation_station_above"])

            # Move die to own belt
            self.send_cart_goal(positions["conveyor_front_above"])
            self.send_cart_goal(positions["conveyor_front_belt"])

            self.send_gripper_goal("open")

            self.send_cart_goal(positions["conveyor_front_above"])

            # If prox sensor doesn't detect die, move to crash position
            if self.current_prox_reading is not True:
                self.get_logger().info("Die not detected!")
                self.send_cart_goal(positions["error"])
                raise SystemExit

            sleep_time = random.uniform(1, 5)

            # Start conveyor
            self.send_conveyor_goal("forward")
            start_time = time_ns()

            sleep(sleep_time)

            self.send_conveyor_goal("stop")
            elapsed_time = (time_ns() - start_time) / 10**9
            publish_msg = Float64()
            publish_msg.data = positions["conveyor_front_above"][
                0
            ] - get_distance_traveled(elapsed_time)

            self.get_logger().info(f"Publishing {publish_msg.data}")

            self.publisher.publish(publish_msg)

            self.send_cart_goal(positions["start"])

        # Exit the program
        self.get_logger().info("Finished!")
        raise SystemExit

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
            (KeyCode(char="s"), bunsen.lab),
        ]
    )
    print("'s': start lab demo")

    try:
        rclpy.spin(bunsen)  # Start executing the node
    except SystemExit:
        print("Quitting...")
        rclpy.shutdown()
