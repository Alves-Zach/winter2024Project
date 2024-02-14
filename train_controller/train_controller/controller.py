import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
import numpy as np
from interbotix_xs_msgs.msg import JointGroupCommand, JointSingleCommand, JointTrajectoryCommand

class controller(Node):
    def __init__(self):
        super().__init__("controller")
        # The subscriber that listens to the sign topic
        self.signSub = self.create_subscription(String, "sign", self.signCallback, 10)

        # Creating known positions for the points along the dial
        self.ctrlPositions = np.array([Point(x=0.25, y=0.25, z=0.25)])

        # Creating the publisher to publish to command the robot
        self.armPub = self.create_publisher(JointGroupCommand, '/px100/commands/joint_group', 10)

        # The publisher to command the gripper
        self.gripperPub = self.create_publisher(JointSingleCommand, '/px100/commands/joint_single', 10)

        # List of joint positions for the commanded velocities
        self.cmdDict = {'0%': [np.deg2rad(7), np.deg2rad(-8), np.deg2rad(45), np.deg2rad(-33)],
                        '20%': [np.deg2rad(9), np.deg2rad(8), np.deg2rad(27), np.deg2rad(-39)]}

        
    def signCallback(self, message):
        # Create the message to send to the joint group command
        msgOut = JointGroupCommand(
            name='arm')
        # Based on the sign detected, make the robot move to a specific point
        if (message.data == "Stop"):
            # Send the action request to move to the desired position
            self.get_logger().info("Stop")
            msgOut.cmd = self.cmdDict['0%']
            self.armPub.publish(msgOut)

        elif (message.data == ""):
            # Go to the home position
            self.get_logger().info("20%")
            msgOut.cmd = self.cmdDict['20%']
            self.armPub.publish(msgOut)

def main(args=None):
    rclpy.init(args=args)
    node = controller()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
