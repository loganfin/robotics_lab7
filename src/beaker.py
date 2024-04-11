# ROS packages
import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_sensor_data
# ROS message typing
from std_srvs.srv import Trigger
from std_msgs.msg import Float64

import sys
sys.path.append("../../dependencies/")
# This is important for running your nodes from the terminal
from pynput.keyboard import KeyCode
from key_commander import KeyCommander

# Fanuc packages
import fanuc_interfaces
from fanuc_interfaces.action import CartPose, SchunkGripper, Conveyor
from fanuc_interfaces.msg import ProxReadings

from time import time_ns, sleep
import random

namespace = 'beaker'

pos1 = [617.0, -3.5, 152.574, 178.0, 0.0, 29.0]
pos2 = [617.0, -3.5, -185.0, 178.0, 0.0, 29.0]
pos3 = [810.0, 642.0, 0.0, 178.0, 0.0, 29.0]
pos4 = [810.0, 642.0, -190.0, 178.0, 0.0, 29.0]
pos5 = [617.0, -300.0, 150.0, 178.152, 0.0, 119.0]
pos6 = [617.0, -300.0, -185.0, 178.152, 0.0, 119.0]
pos7 = [622.0, pos6[1]-100.0, -215.0, 90.0, -57.0, 179.0]
pos8 = [622.0, pos6[1]+5.0, -215.0, 90.0, -57.0, 179.0]
pos9 = [622.0, pos8[1], 150.0, 90.0, -57.0, 179.0]

class FanucSingle(Node):
    def __init__(self, namespace):
        super().__init__("robot")

        # Publisher
        self.publisher_ = self.create_publisher(Float64, 'BenLoganBeaker', 10)

        # Topics
        proxCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.subscriberProx = self.create_subscription(ProxReadings, f'/{namespace}/prox_readings', self.proxListener, qos_profile=qos_profile_sensor_data, callback_group=proxCallbackGroup)
        self.srvProx = self.create_service(Trigger, 'read_conveyor', self.proxServiceCallback)
        self.conveyorLeft = False # Temporary default value

        bunsenCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.subscriberBunsen = self.create_subscription(Float64, '/BenLoganBunsen', self.bunsenListener, qos_profile=qos_profile_sensor_data, callback_group=bunsenCallbackGroup)
        self.receivedBunsonX = False
        self.bunsonX = 0.0

        # Actions
        self.cart_ac = ActionClient(self, CartPose, f'/{namespace}/cartesian_pose')
        self.schunk_ac = ActionClient(self, SchunkGripper, f'/{namespace}/schunk_gripper')  
        self.conveyor_ac = ActionClient(self, Conveyor, f'/{namespace}/conveyor')

        # Sercice Client
        self.service = self.create_client(Trigger,'/read_conveyor')

    def proxListener(self, msg):
      self.conveyorLeft = msg.left

    def proxServiceCallback(self, request, response):
      response.message = str(self.conveyorLeft)
      return response

    def bunsenListener(self, msg):
      print("Bunsen Callback running")
      print(msg.data)
      self.receivedBunsonX = True
      self.bunsonX = msg.data

    def demo(self):
      self.conveyor('stop')

      # Pick up die from starting position
      self.schunkGripper('open')
      self.move(pos1)
      self.move(pos2)
      self.schunkGripper('close')
      self.move(pos1)

      # Place die on conveyor belt and turn it on/off
      self.startPlaceDiceSequence()

      for turnNum in range(2):
        # Wait until this value is changed from receiving a message from Bunsen.
        while self.receivedBunsonX is not True:
          pass
         
        self.receivedBunsonX = False

        aboveDice = [self.bunsonX + 40.0, 895.0, 0.0, 178.0, 0.0, 29.0]
        beakerCoords = [self.bunsonX + 40.0, 895.0, -190.0, 178.0, 0.0, 29.0]

        # Move above dice
        self.move(aboveDice)

        # Grabs dice on conveyor belt
        self.move(beakerCoords)
        self.schunkGripper("close")

        # Move back above dice
        self.move(aboveDice)

        # Rolls die
        self.rollDice()

        # If this is after the second time rolling the die, return to starting position and terminate.
        if(turnNum == 1):
          self.isOver()

        # Place die on conveyor belt and turn it on/off
        self.startPlaceDiceSequence()

    def startPlaceDiceSequence(self):
      # Move to above conveyor belt
      self.move(pos3)
      # Move to place dice on conveyor belt
      self.move(pos4)
      # Place dice
      self.schunkGripper("open")
      # Move back up above conveyor belt
      self.move(pos3)

      # Read prox sensor
      left = self.readConveyorLeft()
      if left != "True":
        print("No dice detected!")
        self.move(pos2)
        return
      
      # Turn on conveyor belt for random time
      self.conveyor('forward')
      startTime = time_ns()
      sleep(random.uniform(1, 5))
      # Turn off conveyor belt
      self.conveyor('stop')
      elapsedTime = (time_ns() - startTime) / 10**9

      # Send topic to Bunsen
      print("Publishing")
      publishMsg = Float64()
      publishMsg.data = pos3[0] - self.getDistanceTraveled(elapsedTime)
      print(publishMsg.data)
      self.publisher_.publish(publishMsg)

      # Move to starting position to wait
      self.move(pos1)

    def isOver(self):
      print("No more turns left")
      #  Move back to starting position
      self.move(pos1)

      #  Move to place dice
      self.move(pos2)

      # Place dice
      self.schunkGripper('open')

      # Return to start position
      self.move(pos1)

      # Exit program
      exit(1)

    def rollDice(self):
      # Place dice on edge of platform
      self.move(pos5)
      self.move(pos6)
      self.schunkGripper("open")
      self.move(pos5)

      # Grab dice from side
      self.move(pos7)
      self.move(pos8)
      self.schunkGripper("close")
      self.move(pos5)

    def getDistanceTraveled(self, timeSlept:float):
       return 107.1 * timeSlept - 36.08

    def move(self, pos:list[float], isBlocking:bool = True):
        print("Moving Robot!")
        self.cart_ac.wait_for_server() # Wait till its ready
        cart_goal = CartPose.Goal() # Make Goal
        # Add all coordinates 
        cart_goal.x = pos[0]
        cart_goal.y = pos[1]
        cart_goal.z = pos[2]
        cart_goal.w = pos[3]
        cart_goal.p = pos[4]
        cart_goal.r = pos[5]

        if isBlocking == True:
            # Send_goal is blocking
            self.cart_ac.send_goal(cart_goal)
        else:
            self.cart_ac.send_goal_async(cart_goal)

        print("Stopped moving robot")

    def schunkGripper(self, command:str):
      print(command, " gripper")
      self.schunk_ac.wait_for_server()
      schunk_goal = SchunkGripper.Goal()
      schunk_goal.command = command
      self.schunk_ac.send_goal(schunk_goal)

    def conveyor(self, command:str):
      print("Commanding conveyor")
      self.conveyor_ac.wait_for_server()
      conveyor_goal = Conveyor.Goal()
      conveyor_goal.command = command
      self.conveyor_ac.send_goal(conveyor_goal)

    def readConveyorLeft(self) -> bool:
      print("Reading conveyor left prox...")
      request = Trigger.Request() # Make a request object
      while not self.service.wait_for_service(timeout_sec=1.0):
        pass # Wait for service to be ready
      return self.service.call(request).message # Send request and block until given a response

if __name__ == '__main__':
    rclpy.init()
	
    fanuc = FanucSingle(namespace)
    # This allows us to start the function once the node is spinning
    keycom = KeyCommander([
		(KeyCode(char='s'), fanuc.demo),
		])
    print("S")
    rclpy.spin(fanuc) # Start executing the node
    rclpy.shutdown()
