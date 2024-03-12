import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
import numpy as np
import time
from interbotix_xs_msgs.msg import JointGroupCommand, JointSingleCommand

class controller(Node):
    def __init__(self):
        super().__init__("controller")
        # The subscriber that listens to the sign topic
        self.signSub = self.create_subscription(String, "sign", self.signCallback, 10)

        # Creating the publisher to publish to command the robot
        self.armPub = self.create_publisher(JointGroupCommand, '/px100/commands/joint_group', 10)

        # The publisher to command the gripper
        self.gripperPub = self.create_publisher(JointSingleCommand, '/px100/commands/joint_single', 10)

        # Current position of the robot
        self.curPosition = 0
        
        # Create the message to send to the joint group command
        self.msgOut = JointGroupCommand(name='arm')

        # The gripper command 
        self.gripperCommand = JointSingleCommand(name='gripper')
        self.gripperCommand.cmd = 250.0

        # Flag to tell if the robot is switching
        # Will ignore all other signs if switching
        self.switching = False

        # List of joint positions for the commanded velocities
        self.cmdDict = {0: [np.deg2rad(8.0), np.deg2rad(6.0), np.deg2rad(78.0), np.deg2rad(-60.0)],
                        20: [np.deg2rad(10.0), np.deg2rad(22.0), np.deg2rad(62.0), np.deg2rad(-60.0)],
                        40: [np.deg2rad(10.0), np.deg2rad(26.0), np.deg2rad(53.0), np.deg2rad(-60.0)],
                        60: [np.deg2rad(-8.0), np.deg2rad(26.0), np.deg2rad(53.0), np.deg2rad(-66.0)],
                        80: [np.deg2rad(-7.0), np.deg2rad(22.0), np.deg2rad(57.0), np.deg2rad(-64.0)],
                        100: [np.deg2rad(-7.0), np.deg2rad(11.0), np.deg2rad(80.0), np.deg2rad(-75.0)]}
        
        self.switchDict = {0: [np.deg2rad(8.0), np.deg2rad(3.0), np.deg2rad(78.0), np.deg2rad(-75.0)], # Move up
                           1: [np.deg2rad(-35.0), np.deg2rad(3.0), np.deg2rad(78.0), np.deg2rad(-75.0)], # Move over
                           2: [np.deg2rad(-36.5), np.deg2rad(75.0), np.deg2rad(-90.0), np.deg2rad(-65.0)], # Go above switch
                           3: [np.deg2rad(-36.5), np.deg2rad(100.0), np.deg2rad(-75.0), np.deg2rad(-65.0)], # Move switch down
                           4: [np.deg2rad(-36.5), np.deg2rad(75.0), np.deg2rad(-90.0), np.deg2rad(-65.0)], # Move up from switch
                           5: [np.deg2rad(8.0), np.deg2rad(3.0), np.deg2rad(78.0), np.deg2rad(-75.0)]} # Move to neutral position
        
        self.moveToStopPose()

        self.homed = False

        self.switched = False

    def moveToPose(self):
        self.gripperCommand.cmd *= -1
        self.gripperPub.publish(self.gripperCommand)
        self.msgOut.cmd = self.switchDict[1]
        self.armPub.publish(self.msgOut)
        time.sleep(2.5)
        self.msgOut.cmd = self.switchDict[2]
        self.armPub.publish(self.msgOut)
        time.sleep(2.5)
        self.msgOut.cmd = self.switchDict[3]
        self.armPub.publish(self.msgOut)
        time.sleep(2.5)

        self.get_logger().info("Moving to Pose")

    def moveToStopPose(self):
        time.sleep(0.85)
        self.gripperPub.publish(self.gripperCommand)
        time.sleep(0.85)
        self.msgOut.cmd = self.cmdDict[self.curPosition]
        self.armPub.publish(self.msgOut)
        time.sleep(0.85)
        self.gripperCommand.cmd *= -1
        self.gripperPub.publish(self.gripperCommand)
        time.sleep(0.85)
        self.homed = True

        self.get_logger().info("Moving to home")
        
    def signCallback(self, message):
        if (self.switching == False):
            # If the message was empty, no sign was seen and nothing should change
            if (message.data != "" and message.data != "Switch"):
                # Get the message as a float
                if (message.data == "Stop"):
                    desiredPosition = 0.0
                    self.msgOut.cmd = self.cmdDict[self.curPosition]
                    self.armPub.publish(self.msgOut)
                    self.get_logger().info("Stopping")
                else:
                    desiredPosition = float(message.data)
                
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
        
            # If the message is switch and the robot isn't currently switching, switch
            if (message.data == "Switch" and self.switched == False):
                self.switchTracks()

    def switchTracks(self):
        self.switching = True

        desiredPosition = 0

        while (self.curPosition != desiredPosition):
            if (self.curPosition > desiredPosition):
                self.curPosition -= 20
                self.msgOut.cmd = self.cmdDict[self.curPosition]
                self.armPub.publish(self.msgOut)
                time.sleep(2.5)
                self.get_logger().info("Current position: " + self.curPosition.__str__())
            else:
                break # Failsafe to make sure the loop breaks when the desired speed and current speed match

        # Robot is now at stopped position
            
        # Release the dial
        self.gripperCommand.cmd = -self.gripperCommand.cmd; # Reversing the commanded value
        self.gripperPub.publish(self.gripperCommand)
        self.get_logger().info("Releasing dial")
        time.sleep(0.85)
        
        # Move up
        self.msgOut.cmd = self.switchDict[0]
        self.armPub.publish(self.msgOut)
        self.get_logger().info("Moving up")
        time.sleep(0.85)

        # Move over
        self.msgOut.cmd = self.switchDict[1]
        self.armPub.publish(self.msgOut)
        self.get_logger().info("Moving over")
        time.sleep(2)

        # Close grippers
        self.gripperCommand.cmd = -self.gripperCommand.cmd; # Reversing the commanded value
        self.gripperPub.publish(self.gripperCommand)
        self.get_logger().info("Closing dial")
        time.sleep(2.5)

        # Go above switch
        self.msgOut.cmd = self.switchDict[2]
        self.armPub.publish(self.msgOut)
        self.get_logger().info("Going above switch")
        time.sleep(2.5)

        # Move switch down
        self.msgOut.cmd = self.switchDict[3]
        self.armPub.publish(self.msgOut)
        self.get_logger().info("Pushing switch down")
        time.sleep(2.5)

        # Move up from switch
        self.msgOut.cmd = self.switchDict[4]
        self.armPub.publish(self.msgOut)
        self.get_logger().info("Move up from switch")
        time.sleep(2.5)

        # Move to neutral position
        self.msgOut.cmd = self.switchDict[5]
        self.armPub.publish(self.msgOut)
        self.get_logger().info("Move up from switch")
        time.sleep(2.5)

        # Open grippers again
        self.gripperCommand.cmd *= -1
        self.gripperPub.publish(self.gripperCommand)
        self.get_logger().info("Opening grippers")
        time.sleep(0.85)

        # Move back to 0 position
        self.curPosition = 0
        self.msgOut.cmd = self.cmdDict[self.curPosition]
        self.armPub.publish(self.msgOut)
        self.get_logger().info("Moving back to 0 position")
        time.sleep(0.85)

        # Grabbing dial again
        self.gripperCommand.cmd *= -1
        self.gripperPub.publish(self.gripperCommand)
        self.get_logger().info("Grabbing dial")
        time.sleep(0.85)

        # Letting program know it can go back to normal
        self.switching = False
        self.switched = True

def main(args=None):
    rclpy.init(args=args)
    node = controller()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
