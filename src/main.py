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

positions = {
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
    # Given a time that the belt was on, return the difference traveled

    # Conveyor belt is about 4ft from beginning to end
    # From the conveyor's product page, the approximate max speed is 17 FPM
    # This seems pretty good for right now for 1 sec
    # return time * 112
    # print(f"Time: {time}, Distance: {103.981 * time - b}")
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

        self.bunsen_subscriber = self.create_subscription(
            Float64,
            "/BenLoganBeaker",
            self.beaker_callback,
            qos_profile_sensor_data,
        )

        # self.prox_sensor = self.create_subscription(
        #     ProxReadings,
        #     f"/{namespace}/prox_readings",
        #     self.prox_sensor_callback,
        #     qos_profile_sensor_data,
        # )

        # self.timer = self.create_timer(time, self.timer_callback)
        # self.timer.cancel()


    def create_subscriber(self):
        self.subscriber = self.create_subscription(
            Float64,
            "/BenLoganBeaker",
            self.beaker_callback,
            qos_profile_sensor_data,
        )

    def bunsen_callback(self, msg):
        self.get_logger().info(f"Bunsen: {msg.data}")

    def beaker_callback(self, msg):
        self.beaker_rx = True

        # Is there an offset that needs to be calculated
        self.beaker_x_coord = msg.data
        self.get_logger().info(f"Beaker: {msg.data}")

    def prox_sensor_callback(self, msg):
        self.current_prox_reading = msg.right

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

    def demo(self):
        self.reset()

        while True:
            self.send_cart_goal(positions["die_start"])

            self.send_gripper_goal("close")

            self.send_cart_goal(positions["start"])

            self.send_cart_goal(positions["conveyor_front_above"])
            self.send_cart_goal(positions["conveyor_front_belt"])

            self.send_gripper_goal("open")

            self.send_cart_goal(positions["conveyor_front_above"])

            time = random.uniform(1, 5)
            # time = 3

            self.get_logger().info(f"Sleeping for {time} seconds")

            self.send_conveyor_goal("forward")
            start_time = time_ns()
            # self.timer.reset()
            sleep(time)
            self.send_conveyor_goal("stop")
            elapsed_time = (time_ns() - start_time) / 10**9

            temp_pos_above = positions["conveyor_front_above_roll"].copy()
            temp_pos_above[0] -= get_distance_traveled(elapsed_time)

            temp_pos_belt = positions["conveyor_front_belt_roll"].copy()
            temp_pos_belt[0] = temp_pos_above[0]

            print(temp_pos_above)

            self.send_cart_goal(temp_pos_above)
            self.send_cart_goal(temp_pos_belt)

            self.send_gripper_goal("close")

            self.send_cart_goal(temp_pos_above)

            self.send_cart_goal(positions["start"])

            self.send_cart_goal(positions["die_start"])

            self.send_gripper_goal("open")

            self.send_cart_goal(positions["start"])

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
        for _ in range(2):
            self.get_logger().info("Waiting on Beaker...")

            # Wait for notification from Beaker
            while self.beaker_rx is not True:
               pass

            self.get_logger().info("Received Message...")
            self.beaker_rx = False

            # Message received, go to Die position
            die_pos_above = positions["conveyor_a_above"].copy()
            # die_pos_above = positions["conveyor_front_above"].copy()

            # Approximately 40mm difference between Beaker and Bunsen
            die_pos_above[0] = self.beaker_x_coord - 55
            # die_pos_above[0] -= 300
            #die_pos_above[0] = 500.0

            die_pos_belt = positions["conveyor_a_belt"].copy()
            # die_pos_belt = positions["conveyor_front_belt"].copy()

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
            # If prox sensor doesn't detect die, move to crash position
            self.send_cart_goal(positions["conveyor_front_above"])
            self.send_cart_goal(positions["conveyor_front_belt"])

            self.send_gripper_goal("open")

            self.send_cart_goal(positions["conveyor_front_above"])

            sleep_time = random.uniform(1, 5)

            # Start conveyor
            self.send_conveyor_goal("forward")
            start_time = time_ns()

            sleep(sleep_time)

            self.send_conveyor_goal("stop")
            elapsed_time = (time_ns() - start_time) / 10**9
            publish_msg = Float64()
            publish_msg.data = positions["conveyor_front_above"][0] - get_distance_traveled(elapsed_time)

            self.get_logger().info(f"Publishing {publish_msg.data}")

            self.publisher.publish(publish_msg)

            self.send_cart_goal(positions["start"])

        # Exit the program
        self.get_logger().info("Finished!")
        raise SystemExit

    def temp(self):
        self.reset()
        self.send_cart_goal(positions["die_start"])
        self.send_gripper_goal("close")
        self.send_cart_goal(positions["rotation_station_above"])
        self.send_cart_goal(positions["rotation_station_table_roll"])
        self.send_gripper_goal("open")
        self.send_cart_goal(positions["rotation_station_above_roll"])
        self.send_cart_goal(positions["rotation_station_side"])
        self.send_cart_goal(positions["rotation_station_push"])
        self.send_gripper_goal("close")
        self.send_cart_goal(positions["rotation_station_above"])

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
            (KeyCode(char="g"), bunsen.temp),
        ]
    )
    print("'t': subscribe to topic")
    print("'s': start die demo")
    print("'l': start lab demo")
    print("'g': start go to temp pos")

    try:
        rclpy.spin(bunsen)  # Start executing the node
    except SystemExit:
        print("Quitting...")
        rclpy.shutdown()
