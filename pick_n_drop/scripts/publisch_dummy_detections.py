#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from depthai_ros_msgs.msg import SpatialDetectionArray
from depthai_ros_msgs.msg import SpatialDetection
from vision_msgs.msg import ObjectHypothesis

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(SpatialDetectionArray, 'color/yolov4_spatial_detections', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.msg = SpatialDetectionArray()
        detection1 = SpatialDetection()
        ObjectHypothesis1 = ObjectHypothesis()
        ObjectHypothesis1.score = 0.85
        ObjectHypothesis1.class_id = str(1)
        detection1.results.append(ObjectHypothesis1)
        detection1.position.x = 0.05
        detection1.position.y = 0.03
        detection1.position.z = 0.30

        detection2 = SpatialDetection()
        ObjectHypothesis2 = ObjectHypothesis()
        ObjectHypothesis2.score = 0.65
        ObjectHypothesis2.class_id = str(2)
        detection2.results.append(ObjectHypothesis2)
        detection2.position.x = 0.04
        detection2.position.y = 0.02
        detection2.position.z = 0.25

        detection3 = SpatialDetection()
        ObjectHypothesis3 = ObjectHypothesis()
        ObjectHypothesis3.score = 0.99
        ObjectHypothesis3.class_id = str(3)
        detection3.results.append(ObjectHypothesis3)
        detection3.position.x = 0.1
        detection3.position.y = 0.04
        detection3.position.z = 0.35

        self.msg.detections.append(detection1)
        self.msg.detections.append(detection2)
        self.msg.detections.append(detection3)

    def timer_callback(self):
        msg = SpatialDetectionArray()
        self.msg.header.frame_id = str(self.i)
        self.msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher_.publish(self.msg)
        self.get_logger().info('Publishing dummydetection: %i' % self.i)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()