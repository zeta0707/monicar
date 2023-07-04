#!/usr/bin/env python
import rclpy
from rclpy.node import Node
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class CameraeNode(Node):
    def __init__(self):

        super().__init__('usbcam_node')
        self.cap = cv2.VideoCapture(1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.image_pub = self.create_publisher(Image, 'webcam_image', 10)
        self.bridge = CvBridge()
        print("Camera Node created")

        # Create a timer that will gate the node actions twice a second
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.node_callback)

    def node_callback(self):
        # Capture frame-by-frame
        ret, cv_image = self.cap.read()
        if ret == True:           
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        else:
            print("image read fail")
            self.cap.release()
            cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args) 
    csiCamera = CameraeNode()
    rclpy.spin(csiCamera)
    csiCamera.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()