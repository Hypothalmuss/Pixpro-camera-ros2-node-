import cv2
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge

class PublisherNodeClass(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.cameraDeviceNumber = 0
        self.camera = cv2.VideoCapture(self.cameraDeviceNumber)
        self.bridgeObject = CvBridge()
        self.topicNameFrames = 'topic_camera_image'
        self.queueSize = 20
        self.publisher = self.create_publisher(Image, self.topicNameFrames, self.queueSize)
        self.periodCommunication = 0.02
        self.timer = self.create_timer(self.periodCommunication, self.timer_callbackFunction)
        self.i = 0

    def timer_callbackFunction(self):
        success, frame = self.camera.read()
        if not success:
            self.get_logger().warning('Failed to capture image')
            return
       
        frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_CUBIC)
       
        if success:
            ROS2ImageMessage = self.bridgeObject.cv2_to_imgmsg(frame)
            self.publisher.publish(ROS2ImageMessage)
            self.get_logger().info('Publishing image number %d' % self.i)
            self.i += 1

    def destroy_node(self):
        self.camera.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    publisherObject = PublisherNodeClass()
    rclpy.spin(publisherObject)
    publisherObject.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
