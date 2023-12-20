#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Image

from sensor_msgs.msg import PointCloud2
import numpy as np

from visualization_msgs.msg import Marker

import ros2_numpy

import cv2
from cv_bridge import CvBridge

import tf2_ros
import geometry_msgs.msg

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf2

class PointcloudTransformer(Node):

    def __init__(self):
        super().__init__('pointcloud_transformer')

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.subPointcloud = self.create_subscription(
            PointCloud2,
            '/stereo/points',
            callback = self.pointcloud_callback,
            qos_profile=qos_profile
        )

        self.pubTransformedPointcloud = self.create_publisher(PointCloud2, '/stereo/points', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    pass
    
    def pointcloud_callback(self, point_cloud_msg):
        self.point_cloud_msg = point_cloud_msg
        try:
            trans = self.tf_buffer.lookup_transform("world", point_cloud_msg.header.frame_id,
                                            point_cloud_msg.header.stamp,
                                            Duration(seconds=10.0))
        except tf2.LookupException as ex:
            #rospy.logwarn(ex)
            return
        except tf2.ExtrapolationException as ex:
            #rospy.logwarn(ex)
            return
        cloud_out = do_transform_cloud(point_cloud_msg, trans)

        self.pubTransformedPointcloud.publish(cloud_out)



def main(args=None):
    rclpy.init(args=args)

    pt = PointcloudTransformer()

    rclpy.spin(pt)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pt.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
