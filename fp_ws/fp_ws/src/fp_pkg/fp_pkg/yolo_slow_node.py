import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32
import numpy as np
from ultralytics import YOLO
import cv2
import os


DETECT_CONFIRM_FRAMES = 3  # N프레임 연속 감지 → 1 발행
LOST_CONFIRM_FRAMES   = 100  # M프레임 연속 미감지 → 0 발행


class NumberDetectedNode(Node):
    def __init__(self):
        super().__init__('number_detected_node')

        cam_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE
        )
        det_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE
        )
        trigger_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE
        )

        self.sub_img = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self.sub_callback, cam_qos
        )
        self.pub_detect = self.create_publisher(Int32, '/slow_detected', det_qos)

        # FSM 트리거 토픽 구독 (다른 노드에서 이 노드를 활성화/비활성화)
        self.sub_trigger = self.create_subscription(
            Int32, '/detect_trigger', self.trigger_callback, trigger_qos
        )

        pkg_path = get_package_share_directory('fp_pkg')
        model_path = os.path.join(pkg_path, 'models', 'best.pt')
        self.model = YOLO(model_path)

        # FSM 활성 상태 (트리거 수신 전까지 카메라 처리 skip)
        self.active = False

        # Debounce 카운터
        self.detect_counter    = 0
        self.lost_counter      = 0
        self.confirmed_detected = False  # 외부로 발행된 확정 상태

    def trigger_callback(self, msg: Int32):
        if msg.data == 1:
            self.active = True
            self._reset_counters()
            self.get_logger().info('FSM trigger received → ACTIVE')
        elif msg.data == 0:
            self.active = False
            self._reset_counters()
            self.get_logger().info('FSM trigger received → INACTIVE')

    def _reset_counters(self):
        self.detect_counter     = 0
        self.lost_counter       = 0
        self.confirmed_detected = False

    def sub_callback(self, msg: CompressedImage):
        # FSM 비활성 상태면 처리 skip (디코딩조차 하지 않음)
        if not self.active:
            return

        np_arr = np.frombuffer(msg.data, np.uint8)
        frame  = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            return

        results = self.model.predict(frame, conf=0.7, imgsz=320, verbose=False)

        detected = any(
            r.boxes is not None and len(r.boxes) > 0
            for r in results
        )
        if detected:
            self.detect_counter += 1
            self.lost_counter    = 0
            if (self.detect_counter >= DETECT_CONFIRM_FRAMES
                    and not self.confirmed_detected):
                self._publish(1)
                self.confirmed_detected = True
                self.get_logger().info('Object CONFIRMED → publish 1\n  객체 식별: 서행 명령 하달')
                self.active = False
                self._reset_counters()
        else:
            self.lost_counter    += 1
            self.detect_counter   = 0
            if (self.lost_counter >= LOST_CONFIRM_FRAMES):
                # self._publish(0)
                self.confirmed_detected = False
                self.get_logger().info('Object LOST 객체 미식별')
                self._reset_counters()

    def _publish(self, value: int):
        out      = Int32()
        out.data = value
        self.pub_detect.publish(out)


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