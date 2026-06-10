import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge


class CameraROSNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.bridge = CvBridge()
        self.img_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.det_pub = self.create_publisher(Detection2DArray, '/detections', 10)

    def publish_frame(self, frame_np):
        if self.img_pub.get_subscription_count() == 0:
            return
        msg = self.bridge.cv2_to_imgmsg(frame_np, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_optical_frame'
        self.img_pub.publish(msg)

    def publish_detections(self, dets):
        msg = Detection2DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_optical_frame'

        for (x1, y1, x2, y2, label, conf) in dets:
            det = Detection2D()
            det.header = msg.header                 
            det.bbox.center.position.x = float((x1 + x2) / 2)
            det.bbox.center.position.y = float((y1 + y2) / 2)
            det.bbox.size_x = float(x2 - x1)
            det.bbox.size_y = float(y2 - y1)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = label
            hyp.hypothesis.score = float(conf)

            det.results.append(hyp)
            msg.detections.append(det)

        self.det_pub.publish(msg)


def init_ros() -> CameraROSNode:
    rclpy.init()
    node = CameraROSNode()

    # SingleThreadedExecutor: this node has 2 publishers and no subscribers, so
    # MultiThreaded adds context-switching overhead without parallelism benefit.
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    threading.Thread(target=executor.spin, daemon=True).start()
    return node