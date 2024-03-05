import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
import numpy as np
import time
from interbotix_xs_msgs.msg import JointGroupCommand, JointSingleCommand
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

class controller(Node):
    def __init__(self):
        super().__init__("controller")
        # The subscriber that listens to the sign topic
        self.signSub = self.create_subscription(String, "sign", self.signCallback, 10)

        # Creating the publisher to publish to command the robot
        self.armPub = self.create_publisher(JointGroupCommand, '/px100/commands/joint_group', 10)

        # The publisher to command the gripper
        self.gripperPub = self.create_publisher(JointSingleCommand, '/px100/commands/joint_single', 10)

        # Defining the gripper pressure
        self.gripperPressure = 250

        # Current position of the robot
        self.curPosition = 0
        
        # Create the message to send to the joint group command
        self.msgOut = JointGroupCommand(name='arm')

        # The gripper command 
        self.gripperCommand = JointSingleCommand(name='gripper')
        self.gripperCommand.cmd = 250.0

        # List of joint positions for the commanded velocities
        self.cmdDict = {0: [np.deg2rad(8.0), np.deg2rad(6.0), np.deg2rad(78.0), np.deg2rad(-67.0)],
                        20: [np.deg2rad(10.0), np.deg2rad(22.0), np.deg2rad(62.0), np.deg2rad(-67.0)],
                        40: [np.deg2rad(10.0), np.deg2rad(26.0), np.deg2rad(53.0), np.deg2rad(-71.0)],
                        60: [np.deg2rad(-8.0), np.deg2rad(26.0), np.deg2rad(53.0), np.deg2rad(-66.0)],
                        80: [np.deg2rad(-7.0), np.deg2rad(22.0), np.deg2rad(57.0), np.deg2rad(-64.0)],
                        100: [np.deg2rad(-7.0), np.deg2rad(11.0), np.deg2rad(80.0), np.deg2rad(-75.0)],}

        
    def signCallback(self, message):
        # If the message was empty, no sign was seen and nothing should change
        if (message.data != "" and message.data != "Switch"):
            # Get the message as a float
            if (message.data == "Stop"):
                desiredPosition = 0.0
                self.msgOut.cmd = self.cmdDict[self.curPosition]
                self.armPub.publish(self.msgOut)
                self.get_logger().info("Stopping")
            else:
                desiredPosition = float(message.data) + 20
            
            self.get_logger().info("Current position: " + self.curPosition.__str__())

            while (self.curPosition != desiredPosition):
                if (self.curPosition > desiredPosition):
                    self.curPosition -= 20
                    self.msgOut.cmd = self.cmdDict[self.curPosition]
                    self.armPub.publish(self.msgOut)
                    time.sleep(0.85)
                    self.get_logger().info("Current position: " + self.curPosition.__str__())
                elif (self.curPosition < desiredPosition):
                    self.curPosition += 20
                    self.msgOut.cmd = self.cmdDict[self.curPosition]
                    self.armPub.publish(self.msgOut)
                    time.sleep(0.85)
                    self.get_logger().info("Current position: " + self.curPosition.__str__())
                else:
                    break # Failsafe to make sure the loop breaks when the desired speed and current speed match
        
        if (message.data == "Switch"):
            self.gripperCommand.cmd = -self.gripperCommand.cmd; # Reversing the commanded value
            self.gripperPub.publish(self.gripperCommand)
            self.get_logger().info("Ahhhhhhhh")
            time.sleep(2.5)

def main(args=None):
    rclpy.init(args=args)
    node = controller()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
