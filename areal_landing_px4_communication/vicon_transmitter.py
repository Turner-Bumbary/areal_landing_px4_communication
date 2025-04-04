
'''
Written by Adam Garlow

This node follows a straightforward sub/pub framework to take mocap messages
and publish them on the desired px4 microRTPS bridge topic

MoCap messages received on topic: '/vicon/X500_v2_IRcam/X500_v2_IRcam'

PX4 messages published on topic: '/fmu/vehicle_visual_odometry/in'

'''

# Import Packages
import rclpy
from rclpy.node import Node

# Import ROS Messages
from geometry_msgs.msg import Point
from vicon_receiver.msg import Position

# Pubsub node class definition
class ViconTransmitter(Node):

    # Pubsub constructor
    def __init__(self):
        super().__init__('vicon_transmitter') # Initialize node

        # Initialize subscriber to the Vicon topic
        self.mocap_sub = self.create_subscription(Position, 
            '/vicon/Turner_X500/Turner_X500', self.mocap_callback, 10)

        # Initilize publisher for motion capture data on /position topic
        self.position_pub = self.create_publisher(Point, 
            '/position', 10)

        self.get_logger().info("Vicon transmitter node initialized")


    # Callback for when new mocap message recieved to publish to PX4
    def mocap_callback(self, msg):  
        # Create point message for position      
        position = Point()
        position.x = msg.x_trans/1000.0
        position.y = msg.y_trans/1000.0
        position.z = msg.z_trans/1000.0
        
        self.position_pub.publish(position)
        
def main(args=None):
    rclpy.init(args=args)

    vicon_transmitter = ViconTransmitter()

    rclpy.spin(vicon_transmitter)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
