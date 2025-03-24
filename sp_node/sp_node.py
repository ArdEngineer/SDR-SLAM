import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import ByteMultiArray
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import pickle

from sp_net_core import *

fe = SuperPointFrontend(weights_path='./superpoint_v1.pth',
                          nms_dist=10,
                          conf_thresh=0.08,
                          nn_thresh=0.5,
                          cuda=True)

class FeatureExtractorNode(Node):
    def __init__(self):
        super().__init__('feature_extractor_node')
        self.subscription = self.create_subscription(
            Image,
            'image_from_featrueTracker',
            self.image_callback,
            10)
        # self.publisher = self.create_publisher(Float32MultiArray, 'features_data', 10)
        self.keypoints_publisher = self.create_publisher(Float32MultiArray, 'keypoints_topic', 10)
        self.descriptors_publisher = self.create_publisher(Float32MultiArray, 'descriptors_topic', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        self.get_logger().info('Received image data from featureTracker')
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')  # 灰度图像
        cv_image_float32 = cv_image.astype(np.float32) / 255.0 
        keypoints, descriptors, heatmap = fe.run(cv_image_float32)

        print(descriptors)

         # 提取特征点坐标
        keypoints_coords = keypoints[:2, :].T.flatten().tolist()  # 只取前两列（x, y）

        # 将描述子转换为一维数组
        descriptors_array = descriptors.T.flatten().tolist()

        # 创建 keypoints 消息并发布
        keypoints_msg = Float32MultiArray()
        keypoints_msg.data = keypoints_coords
        self.keypoints_publisher.publish(keypoints_msg)

        # 创建 descriptors 消息并发布
        descriptors_msg = Float32MultiArray()
        descriptors_msg.data = descriptors_array
        self.descriptors_publisher.publish(descriptors_msg)

        self.get_logger().info('Published keypoints and descriptors')

def main(args=None):
    rclpy.init(args=args)
    node = FeatureExtractorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()