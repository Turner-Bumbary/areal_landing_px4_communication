
'''
Written by Turner Bumbary

This node subscribes to the Vicon messages to visualize the pose of the
vehicle in RViz.

Vicon messages received on topic: '/vicon/Turner_X500/Turner_X500'

'''

# Standard library imports
import rclpy
from rclpy.node import Node


# Third-party imports
from tf2_ros import TransformBroadcaster

from vicon_receiver.msg import Position
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from builtin_interfaces.msg import Duration
from rclpy.qos import qos_profile_sensor_data

import numpy as np
from numpy import nan as NaN

# Pubsub node class definition
class PoseVisualization(Node):

    # Pubsub constructor
    def __init__(self):
        super().__init__('pose_visualization') # Initialize node

        # Initialize subscriber to mocap(VICON) topic
        self.mocap_sub = self.create_subscription(Position, 
            '/vicon/Turner_X500/Turner_X500', self.mocap_callback, 10)
        
        # Initialize publisher for RViz visualization w/ marker
        self.marker_pub = self.create_publisher(MarkerArray, '/markers', 10)
        self.marker_array = MarkerArray()
        self.marker_array_id = 0
        
        # Initialize transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("Vicon pose visualization node initialized")


    # Callback function for vicon messages
    def mocap_callback(self, msg):        
        # Create a transform message
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'drone'
        t.transform.translation.x = msg.x_trans/1000
        t.transform.translation.y = msg.y_trans/1000
        t.transform.translation.z = msg.z_trans/1000
        t.transform.rotation.w = msg.w
        t.transform.rotation.x = msg.x_rot
        t.transform.rotation.y = msg.y_rot
        t.transform.rotation.z = msg.z_rot
    
        # Publish the transform    
        self.tf_broadcaster.sendTransform(t)
        
        # Create a marker message        
        marker = Marker()
        marker.id = self.marker_array_id
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'drone_markers'
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.g = 1.0
        marker.color.a = 1.0
        marker.lifetime.sec = 2
        marker.pose.position.x = msg.x_trans/1000
        marker.pose.position.y = msg.y_trans/1000
        marker.pose.position.z = msg.z_trans/1000
        
        self.marker_array_id = self.marker_array_id + 1
        self.marker_array.markers.append(marker)
        
        # Publish the marker msg
        self.marker_pub.publish(self.marker_array)
        
        

def main(args=None):
    rclpy.init(args=args)

    pose_visualizer = PoseVisualization()

    rclpy.spin(pose_visualizer)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
