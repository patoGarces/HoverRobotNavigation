import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import queue
from ros2_ps5_stereo.cameraPs5Handler import CameraPs5Handler

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
       
        self.cameraPs5Handler = CameraPs5Handler()
        self.frame_queue = self.cameraPs5Handler.getQueueFrames()
        self.bridge = CvBridge()

        # Publishers para cada cámara (izquierda y derecha)
        self.pub_left = self.create_publisher(Image, 'camera_left/image_raw', 10)
        self.pub_right = self.create_publisher(Image, 'camera_right/image_raw', 10)

        self.queueFrames = self.cameraPs5Handler.getQueueFrames()

        self.timer = self.create_timer(0.033, self.framePublisher)  # ~30Hz


    def framePublisher(self):
        try:
            frames = self.frame_queue.get_nowait()  # sin bloqueo
        except queue.Empty:
            return  # no hay frames, salimos

        frameR, frameL = frames

        # Convertir OpenCV a Image ROS
        msg_left = self.bridge.cv2_to_imgmsg(frameL, encoding='bgr8')
        msg_right = self.bridge.cv2_to_imgmsg(frameR, encoding='bgr8')

        # Publicar
        self.pub_left.publish(msg_left)
        self.pub_right.publish(msg_right)



def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
