import rclpy
#import math
#import numpy as np

from rclpy.node import Node

#from geometry_msgs.msg import Twist
#from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from rclpy.qos import ReliabilityPolicy, QoSProfile
#from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
#from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor



class Mapping(Node):

    def __init__(self,odom_topic="/odom"):

        self._odom_topic = odom_topic
        
        self.x_pos = None
        self.y_pos = None

       

        super().__init__('mapping')

        rclpy.logging.set_logger_level('movement_server', rclpy.logging.LoggingSeverity.INFO)


        self.odom_subscriber= self.create_subscription(Odometry,self._odom_topic, self.position,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

    def position(self, msg):
        self.x_pos = msg.pose.pose.position.x
        self.y_pos = msg.pose.pose.position.y
        self.get_logger().info('I receive X: "%s"' % str(self.x_pos))
        self.get_logger().info('I receive Y: "%s"' % str(self.y_pos))

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    mapping = Mapping()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(mapping)
    # Explicity destroy the node
    mapping.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()

