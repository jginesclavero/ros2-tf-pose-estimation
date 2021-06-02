#!/usr/bin/env python
import time
import os
import sys
import ast
from threading import Lock

from numpy import double
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image, CameraInfo
from tf_pose_estimation_msgs.msg import Frame, Person, BodyPart, Pixel

from tf_pose.estimator import TfPoseEstimator
from tf_pose.networks import model_wh, get_graph_path


class TFPoseEstimationNode(Node):
    def __init__(self, last_contexts=None):
        """Initialize all params and load data."""
        """ Constants and params """
        super().__init__('tf_pose_estimation_node')
        parameters = [
            ('image_topic', '', ParameterDescriptor()),
            ('depth_image_topic', '', ParameterDescriptor()),
            ('camera_info_topic', '', ParameterDescriptor()),
            ('model', 'cmu', ParameterDescriptor()),
            ('resolution', '432x368', ParameterDescriptor()),
            ('resize_out_ratio', 4.0, ParameterDescriptor()),
        ]
        self.declare_parameters('', parameters)
        image_topic = self.get_parameter_or('image_topic', 
            Parameter('image_topic', type_ = Parameter.Type.STRING, value = ""))._value
        depth_image_topic = self.get_parameter_or('depth_image_topic', 
            Parameter('depth_image_topic', type_ = Parameter.Type.STRING, value = ""))._value
        camera_info_topic = self.get_parameter_or('camera_info_topic', 
            Parameter('camera_info_topic', type_ = Parameter.Type.STRING, value = ""))._value
        model = self.get_parameter_or('model', 
            Parameter('model', type_ = Parameter.Type.STRING, value = ""))._value
        resolution = self.get_parameter_or('resolution', 
            Parameter('resolution', type_ = Parameter.Type.STRING, value = ''))._value
        self._resize_out_ratio = self.get_parameter_or('resize_out_ratio', 
            Parameter('resize_out_ratio', type_ = Parameter.Type.DOUBLE, value = 4.0))._value
        self._tf_lock = Lock()
        self.DISTANCE_INFO = False

        if not image_topic:
            self.get_logger().error('Parameter \'image_topic\' is not provided.')
            sys.exit(-1)

        try:
            w, h = model_wh(resolution)
            graph_path = get_graph_path(model)
            graph_path = os.path.join(get_package_share_directory('tf_pose_estimation'), graph_path)
        except Exception as e:
            self.get_logger().error('invalid model: %s, e=%s' % (model, e))
            sys.exit(-1)
        self._image_msg = Image()
        self._pose_estimator = TfPoseEstimator(graph_path, target_size=(w, h))
        self._cv_bridge = CvBridge()
        
        self._pose_pub = self.create_publisher(Frame, "pose", 1)
        self._image_sub = self.create_subscription(Image, image_topic, self.image_cb, 10)
        if depth_image_topic and camera_info_topic:
            self._depth_image_sub = self.create_subscription(
                Image, depth_image_topic, self.depth_image_cb, 1)
            self._camera_info_sub = self.create_subscription(
                CameraInfo, camera_info_topic, self.camera_info_cb, 1)
            self._depth_image_msg = Image()
            self.DISTANCE_INFO = True
            self._camera_info_ready = False

        self.get_logger().info("Subscribe to " + image_topic)
        self.get_logger().info("Ready!")

    def destroy(self):
        super().destroy_node()

    def camera_info_cb(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self._camera_info_ready = True

    def depth_image_cb(self, msg):
        self._depth_image_msg = msg

    def image_cb(self, msg):
        self._image_msg = msg
    
    def humans_to_msg(self, humans):
        frame = Frame()
        for human in humans:
            person = Person()            
            for k in human.body_parts:
                body_part = human.body_parts[k]
                if body_part.score > 0.55:
                    body_part_msg = BodyPart()
                    pixel = Pixel()
                    body_part_msg.id = str(body_part.part_idx)
                    pixel.x = int(body_part.x * self._image_msg.width + 0.5)
                    pixel.y = int(body_part.y * self._image_msg.height + 0.5)
                    body_part_msg.pixel = pixel
                    body_part_msg.score = body_part.score
                    person.body_parts.append(body_part_msg)
            frame.persons.append(person)
        return frame

    def get_2d_pose(self):
        msg = Frame()

        try:
            cv_image = self._cv_bridge.imgmsg_to_cv2(self._image_msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error('[tf-pose-estimation] Converting Image Error. ' + str(e))
            return msg

        acquired = self._tf_lock.acquire(False)
        if not acquired:
            return msg

        try:
            humans = self._pose_estimator.inference(
                cv_image, resize_to_default=True, upsample_size=self._resize_out_ratio)
        finally:
            self._tf_lock.release()
            
        msg = self.humans_to_msg(humans)
        msg.image_w = self._image_msg.width
        msg.image_h = self._image_msg.height
        msg.header = self._image_msg.header
        return msg
    
    def get_3d_pose(self, frame): 
        output_part = BodyPart()
        o_frame = Frame()
        try:
            cv_image = self._cv_bridge.imgmsg_to_cv2(self._depth_image_msg, "32FC1") #16UC1
            #cv_image_norm = cv2.normalize(cv_image, cv_image, 0, 1, cv2.NORM_MINMAX)
        except CvBridgeError as e:
            self.get_logger().error('[tf-pose-estimation] Converting Depth Image Error. ' + str(e))
            return o_frame
        if self._camera_info_ready:
            for person in frame.persons:
                parts = []
                for part in person.body_parts:    
                    output_part = part
                    depth = float(cv_image[part.pixel.x, part.pixel.y])
                    #center_coordinates = (part.pixel.x, part.pixel.y)
                    #cv_image_norm = cv2.circle(cv_image_norm, center_coordinates, 2, (255, 0, 0), 1)
                    output_part.point.x = (
                        ((part.pixel.x - self.cx) * depth) / self.fx)
                    output_part.point.y = (
                        ((part.pixel.y - self.cy) * depth) / self.fy)
                    output_part.point.z = depth
                    parts.append(output_part)
                person.body_parts = parts
                o_frame.persons.append(person)
        #cv2.imshow("Image window", cv_image)
        #cv2.waitKey(1)
        o_frame.image_w = frame.image_w
        o_frame.image_h = frame.image_h
        o_frame.header = frame.header
        return o_frame

    def publish_pose(self):
        msg = Frame()
        self.frame_2d_msg = self.get_2d_pose()
        msg = self.frame_2d_msg
        if self.DISTANCE_INFO:
            msg = self.get_3d_pose(self.frame_2d_msg)
        self._pose_pub.publish(msg)

    def start(self):
        """Start the tf_pose_estimation node"""
        self.get_logger().info("Spinning...")
        while rclpy.ok():
            self.publish_pose()
            rclpy.spin_once(self, timeout_sec=0.033)
       

def main(args=None):
    rclpy.init(args=args)
    node = TFPoseEstimationNode()
    try:
        node.start()
    except KeyboardInterrupt:
        pass

    node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()