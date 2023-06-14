#!/usr/bin/env python
import rclpy
from rclpy.node import Node
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def gstreamer_pipeline(
    capture_width=640,
    capture_height=480,
    display_width=640,
    display_height=480,
    framerate=60,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

class CameraeNode(Node):
    def __init__(self):

        super().__init__('camera_node')

        #for csi camera orientation
        cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
        self.image_pub = self.create_publisher(Image, 'csi_image', 10)

        bridge = CvBridge()

        # Capture frame-by-frame
        ret, cv_image = cap.read()

        # Display the resulting frame
        # cv2.imshow('frame',cv_image)
        # cv2.waitKey(3)
        self.image_pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))

        # When everything done, release the capture
        cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args) 
    csiCamera = CameraeNode()
    rclpy.spin(csiCamera)
    csiCamera.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()