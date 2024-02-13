import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class controller(Node):
    def __init__(self):
        super().__init__('controller')
        # The subscriber that listens to the sign topic
        self.signSub = self.create_subscription(String, "sign", self.signCallback, 10)
        
        
    def signCallback(self, message):
        # Based on the sign detected, make the robot move to a specific point
        if (message.data == "stop"):
            # Move the arm to the far right
            print("wew")

def main(args=None):
    rclpy.init(args=args)
    node = controller()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
