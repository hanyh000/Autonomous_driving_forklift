#!/usr/bin/env python3
import threading
import time
import cv2
import numpy as np
import rclpy
import serial
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String

# -- 설정값 --
_SERIAL_PORT     = "/dev/ttyACM2"
_SERIAL_BAUD     = 9600
_FORK_PIN_UP     = 8
_FORK_PIN_DOWN   = 9
_FORK_MOTION_SEC = 1.5

class ForkController:
    def __init__(self, ser: serial.Serial, logger) -> None:
        self._ser    = ser
        self._logger = logger

    def _send_pin(self, pin: int) -> None:
        try:
            self._ser.write(f"{pin}\n".encode())
            self._logger.info(f"[Arduino] 핀 {pin} 신호 전송")
        except serial.SerialException as e:
            self._logger.error(f"[Arduino] 전송 오류: {e}")

    def fork_up(self) -> None:
        self._logger.info("[Fork] UP")
        self._send_pin(_FORK_PIN_UP)
        time.sleep(_FORK_MOTION_SEC)

    def fork_down(self) -> None:
        self._logger.info("[Fork] DOWN")
        self._send_pin(_FORK_PIN_DOWN)
        time.sleep(_FORK_MOTION_SEC)

class IntegratedForkliftNode(Node):
    LIN_SPD     = 0.06
    MIS_LIN_SPD = 0.05
    ANG_SPD     = 0.4
    P2M         = 0.00075
    TURN_CONST  = 2.95
    DEADZONE_M  = 0.001
    TARGET_Y    = 336
    ALLOWED_IDS = {0, 1, 2}

    def __init__(self) -> None:
        super().__init__("integrated_forklift_node")
        try:
            self._ser = serial.Serial(_SERIAL_PORT, _SERIAL_BAUD, timeout=2)
            time.sleep(2.0)
            self.get_logger().info(f"✅ Serial Connected: {_SERIAL_PORT}")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Serial Error: {e}")
            raise SystemExit

        self._fork   = ForkController(self._ser, self.get_logger())
        self._bridge = CvBridge()

        _dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        _params = cv2.aruco.DetectorParameters()
        self._detector = cv2.aruco.ArucoDetector(_dict, _params)

        self._is_running        = False
        self._marker_found      = False
        self._error_px          = 0
        self._marker_top_y      = 0
        self._last_update_time  = 0.0
        self._current_target_id: int | None = None

        qos_img = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos = QoSProfile(depth=10)

        self.create_subscription(Image,  "/image_raw",  self._img_callback,      qos_img)
        self.create_subscription(Int32,  "/carry",      self._carry_callback,    qos)
        self.create_subscription(String, "/fork_cmd",   self._fork_cmd_callback, qos)

        self._pub_vel        = self.create_publisher(Twist,  "/cmd_vel",    qos)
        self._pub_carry_done = self.create_publisher(String, "/carry_done", qos)
        self._pub_fork_done  = self.create_publisher(String, "/fork_done",  qos)

        self.get_logger().info("[TEST] 노드 시작 — 포크 UP 초기화")
        self._fork.fork_up()
        self._publish_fork_done("UP_DONE")

    def _img_callback(self, msg: Image) -> None:
        if self._current_target_id is None: return
        try:
            img = self._bridge.imgmsg_to_cv2(msg, "bgr8")
            center_x = img.shape[1] // 2
            corners, ids, _ = self._detector.detectMarkers(img)
            if ids is not None:
                for i, m_id in enumerate(ids.flatten()):
                    if m_id == self._current_target_id:
                        self._marker_top_y = np.min(corners[i][0][:, 1])
                        self._error_px = int(np.mean(corners[i][0][:, 0])) - center_x
                        self._marker_found = True
                        self._last_update_time = time.time()
                        return
            self._marker_found = False
        except: pass

    def _carry_callback(self, msg: Int32) -> None:
        tid = msg.data
        if tid not in self.ALLOWED_IDS: return
        if self._is_running: return
        self._current_target_id = tid
        threading.Thread(target=self._run_full_sequence, daemon=True).start()

    def _fork_cmd_callback(self, msg: String) -> None:
        cmd = msg.data.strip().upper()
        if cmd == "UP":
            self._fork.fork_up()
            self._publish_fork_done("UP_DONE")
        elif cmd == "DOWN":
            self._fork.fork_down()
            self._publish_fork_done("DOWN_DONE")

    def _stop(self) -> None:
        self._pub_vel.publish(Twist())
        time.sleep(0.5)

    def _timed_move(self, linear: float, angular: float, duration: float) -> None:
        twist = Twist()
        twist.linear.x, twist.angular.z = float(linear), float(angular)
        end = time.time() + duration
        while time.time() < end and rclpy.ok() and self._is_running:
            self._pub_vel.publish(twist)
            time.sleep(0.05)
        self._stop()

    # ❗ 들여쓰기 및 위치 수정됨
    def _find_zero_point(self, target_y: int, timeout: float = 15.0) -> None:
        self.get_logger().info(f"📍 Zero-point Search Started (Target Y: {target_y})")
        start = time.time()
        P_GAIN, MIN_SPD = 0.0008, 0.02
        
        while rclpy.ok() and self._is_running:
            if (time.time() - start) > timeout:
                raise TimeoutError("Zero-point timeout")
            if not self._marker_found:
                self._stop()
                continue

            diff = target_y - self._marker_top_y
            if abs(diff) <= 2:
                self.get_logger().info(f"✅ Reached: {self._marker_top_y}")
                break

            target_speed = diff * P_GAIN
            direction = 1.0 if diff > 0 else -1.0
            abs_speed = max(MIN_SPD, min(abs(target_speed), self.LIN_SPD))
            
            twist = Twist()
            twist.linear.x = abs_speed * direction
            self._pub_vel.publish(twist)
            time.sleep(0.05)
        self._stop()

    def _wait_fresh(self) -> float | None:
        time.sleep(1.0)
        if self._marker_found and (time.time() - self._last_update_time < 0.5):
            return self._error_px * self.P2M
        return None

    def _publish_fork_done(self, result: str) -> None:
        self._pub_fork_done.publish(String(data=result))

    def _publish_carry_done(self, result: str) -> None:
        self._pub_carry_done.publish(String(data=result))

    def _run_full_sequence(self) -> None:
        self._is_running = True # 루프 시작 전 반드시 True 설정
        try:
            self._find_zero_point(self.TARGET_Y)
            
            # 정렬 및 주행 시퀀스 (기존 로직 유지)
            for _ in range(2):
                err_m = self._wait_fresh()
                if err_m is None: raise RuntimeError("Marker lost")
                if abs(err_m) < self.DEADZONE_M: break
                rot_dir = -1.0 if err_m > 0 else 1.0
                t_rot, t_side = self.TURN_CONST/self.ANG_SPD, abs(err_m)/self.LIN_SPD
                self._timed_move(0.0, self.ANG_SPD*rot_dir, t_rot)
                self._timed_move(self.LIN_SPD, 0.0, t_side)
                self._timed_move(0.0, -self.ANG_SPD*rot_dir, t_rot)

            self._timed_move(self.MIS_LIN_SPD, 0.0, 8.0)
            self._fork.fork_down()
            self._publish_fork_done("DOWN_DONE")
            self._timed_move(-self.MIS_LIN_SPD, 0.0, 8.0)
            self._fork.fork_up()
            self._publish_fork_done("UP_DONE")
            self._publish_carry_done(f"success: ID {self._current_target_id}")

        except Exception as e:
            self.get_logger().error(f"❌ Error: {e}")
            self._publish_carry_done(f"fail: {e}")
        finally:
            self._stop()
            self._is_running = False
            self._current_target_id = None

    def destroy_node(self) -> None:
        if self._ser.is_open: self._ser.close()
        super().destroy_node()

def main(args=None) -> None:
    rclpy.init(args=args)
    node = IntegratedForkliftNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
