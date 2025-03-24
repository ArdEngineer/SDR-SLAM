#!/usr/bin/env python3.8
import torch.cuda
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import os
import numpy as np

from PIL import Image as PIL_IMG

import torch

import net_core

class Enhancd_Modules(Node):
    def __init__(self, CAM_TOPIC):
        super().__init__('imge_enhance_processor')
        self.get_logger().info('[INFO] Image processor initialize.')

        # Create camera image subscriber
        self.CAM_img_sub = self.create_subscription(
            Image,
            CAM_TOPIC,
            self.img_callback,
            10
        )

        # Create enhanced image publisher
        self.EIMG_PUB = self.create_publisher(
            Image,
            'EIMG',
            10
        )

        # Pre-generate the net_core model
        self.DCE = net_core.enhance_net_nopool().cuda()
        self.DCE.load_state_dict(torch.load('snapshots/Epoch99.pth'))

        # Image data transform function
        self.bridge = CvBridge()

    def img_callback(self, CAM_IMAGE):
        self.enhance_processor(CAM_IMAGE)

    def enhance_processor(self, image):
        self.get_logger().info("Receive img from cam")
        
        cv_image = image

        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().info(e)
        header = image.header

        os.environ['CUDA_VISIBLE_DEVICES']='0'

        pil_image = PIL_IMG.fromarray(cv_image)
        
        pil_image = (np.asarray(pil_image) / 255.0)
        pil_image = torch.from_numpy(pil_image).float()
        pil_image = pil_image.permute(2,0,1)
        pil_image = pil_image.cuda().unsqueeze(0)

        _, enhanceImg, _ = self.DCE(pil_image)
        torch.cuda.empty_cache()
        enhanceImg = enhanceImg.detach().cpu().numpy()

        enhanceImg = np.transpose(enhanceImg, (2, 3, 1, 0))
        enhanceImg = np.squeeze(enhanceImg)

        enhanceImg = (enhanceImg - np.min(enhanceImg)) / (np.max(enhanceImg) - np.min(enhanceImg)) * 65535
        enhanceImg = np.uint16(enhanceImg)

        enhanceImg = cv2.cvtColor(enhanceImg, cv2.COLOR_RGB2GRAY)

        try:
            ros_image = self.bridge.cv2_to_imgmsg(enhanceImg, 'mono16')
        except CvBridgeError as e:
            self.get_logger().info(e)
        ros_image.header = header

        self.EIMG_PUB.publish(ros_image)
    
def main(args=None):
    rclpy.init(args=args)
    node = Enhancd_Modules('cam0/image_raw')
    print("[INFO] Node run successfully\n")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
