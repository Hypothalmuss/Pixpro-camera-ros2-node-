import cv2
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2.aruco as aruco
import yaml
import numpy as np

class SubscriberNodeClass(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.bridgeObject = CvBridge()
        self.topicNameFrames = 'topic_camera_image'
        self.queueSize = 20
        self.subscription = self.create_subscription(
            Image,
            self.topicNameFrames,
            self.listener_callbackFunction,
            self.queueSize
        )
       
        # Load camera calibration parameters
        with open('camera_calibration.yaml') as f:
            calib_data = yaml.safe_load(f)
            self.camera_matrix = np.array(calib_data['camera_matrix'])
            self.dist_coeff = np.array(calib_data['dist_coeff'])

    def listener_callbackFunction(self, imageMessage):
        self.get_logger().info('The image frame is received')
        openCVImage = self.bridgeObject.imgmsg_to_cv2(imageMessage)

        # Undistort the image
        openCVImage = cv2.undistort(openCVImage, self.camera_matrix, self.dist_coeff, None, self.camera_matrix)

        openCVImageGray = cv2.cvtColor(openCVImage, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(openCVImageGray, aruco_dict, parameters=parameters)
        aruco.drawDetectedMarkers(openCVImage, corners, ids)
        cv2.imshow("camera video", openCVImage)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    subscriberNode = SubscriberNodeClass()
    rclpy.spin(subscriberNode)
    subscriberNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

