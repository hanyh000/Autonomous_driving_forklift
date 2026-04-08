import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import CompressedImage
import numpy as np 
from ultralytics import YOLO
import cv2
import os

class NumberDetectedNode(Node):
    def __init__(self):
        super().__init__('number_detected_node')
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        self.bridge = CvBridge()
        self.sub_img = self.create_subscription(CompressedImage, '/image_raw/compressed', self.sub_callback, qos)
        self.pub_img = self.create_publisher(CompressedImage, '/numberDetected/compressed', qos)

        pkg_path = get_package_share_directory('fp_pkg')
        model_path = os.path.join(pkg_path, 'models', 'best.pt')
        self.model = YOLO(model_path)

    def sub_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        if frame is None:
            return

        results = self.model.predict(
            frame,
            conf=0.7,
            imgsz=320,
            verbose=False
        )

        for r in results:
            if r.boxes is not None:
                for box in r.boxes:
                    class_id = int(box.cls[0])
                    class_name = self.model.names[class_id]
                    conf = float(box.conf[0])
                    x1, y1, x2, y2 = map(int, box.xyxy[0])

                    self.get_logger().info(f'DETECTED digit={class_name}, conf={conf:.2f}')

                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(
                        frame, 
                        f'{class_name} {conf:.2f}', 
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        0.7, 
                        (0, 255, 0), 
                        2
                    )
        print("====== frame ===========")
        _, buffer = cv2.imencode('.jpg', frame)
        msg_out = CompressedImage()
        msg_out.header = msg.header 
        msg_out.format = "jpeg"
        msg_out.data = np.array(buffer).tobytes()

        self.pub_img.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = NumberDetectedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()