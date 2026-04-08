#!/usr/bin/env python3
"""
integrated_forklift_node.py
ROS2 Humble — ArUco 마커 추적 + 포크 제어 통합 노드
수평 정렬(Yaw Alignment) + 섹터 탐색 추가 버전

[수정 사항]
1. _img_callback: 쓰로틀 타이머(_last_calc_time)와 감지 성공 타이머(_last_found_time) 분리
2. _align_yaw / _find_zero_point: 0.15초 신선도 체크 추가
3. _timed_move: min(0.02, remaining) 슬립 + 실제 경과 시간 기준 종료
4. _stop: 정지 명령 0.3초간 반복 전송

Step 0-0  [섹터 탐색] 마커 미감지 시 우회전 90° → 20cm 전진 → 좌회전 90° → 5초 대기
          수평 정렬 1회
Step 0-1  마커 최초 발견 대기 (10초 타임아웃)
Step 0-2  수직 거리 정렬 - marker_top_y → TARGET_Y ±10px (무제한)
Step 1-1  [섹터 탐색] 마커 미감지 시 우회전 90° → 20cm 전진 → 좌회전 90° → 5초 대기
          수평 정렬 1회 → 좌우 정렬 크랩 워킹 1회차
Step 1-2  수평 정렬 1회 → 좌우 정렬 크랩 워킹 2회차
Step 2    전진
Step 3    포크 UP
Step 4    후진
"""

import threading
import time
import cv2
import numpy as np
import rclpy
import serial
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String

# ── 설정값 ──────────────────────────────────────
_SERIAL_PORT     = "/dev/ttyACM2"
_SERIAL_BAUD     = 9600
_FORK_PIN_UP     = 8
_FORK_PIN_DOWN   = 9
_FORK_MOTION_SEC = 1.5

# ── Arduino 포크 제어 ────────────────────────────
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
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()
        self._send_pin(_FORK_PIN_UP)
        time.sleep(_FORK_MOTION_SEC)

    def fork_down(self) -> None:
        self._logger.info("[Fork] DOWN")
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()
        self._send_pin(_FORK_PIN_DOWN)
        time.sleep(_FORK_MOTION_SEC)


# ── 통합 노드 ────────────────────────────────────
class IntegratedForkliftNode(Node):

    # ── 파라미터 ──
    LIN_SPD         = 0.06    # 크랩 워킹 직진 속도
    MIS_LIN_SPD     = 0.05    # 미션 전진/후진/섹터 탐색 속도
    ZERO_LIN_SPD    = 0.03    # 수직 거리 정렬 저속 (오버슈트 방지)
    YAW_LIN_SPD     = 0.05    # 수평 정렬 저속 (동일 논리 적용)
    ANG_SPD         = 0.4
    P2M             = 0.00075
    TURN_CONST      = 2.95
    DEADZONE_M      = 0.001   # 좌우 정렬 데드존 1mm
    YAW_DEADZONE    = 0.8     # 수평 정렬 데드존 (픽셀)
    TARGET_Y        = 336
    ALLOWED_IDS     = {0, 1, 2}

    # 섹터 탐색: MIS_LIN_SPD(0.05) 기준 30cm = 6.0초
    SECTOR_FWD_TIME = 6

    # ── [수정 1] 신선도 기준 (초) ──
    FRESH_THRESHOLD = 0.15    # 정렬 루프에서 허용하는 최대 프레임 나이

    def __init__(self) -> None:
        super().__init__("integrated_forklift_node")

        # ── 시리얼 연결 ──
        try:
            self._ser = serial.Serial(_SERIAL_PORT, _SERIAL_BAUD, timeout=2)
            time.sleep(2.0)
            self.get_logger().info(f"✅ Serial Connected: {_SERIAL_PORT}")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Serial Error: {e}")
            raise SystemExit

        self._fork   = ForkController(self._ser, self.get_logger())
        self._bridge = CvBridge()

        # ── ArUco 디텍터 설정 ──
        _dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        _params = cv2.aruco.DetectorParameters()
        self._detector = cv2.aruco.ArucoDetector(_dict, _params)

        # ── 내부 상태 변수 ──
        self._is_running              = False
        self._marker_found            = False   # target ID 마커 감지 여부
        self._any_marker_found        = False   # ID 무관 마커 감지 여부 (yaw 정렬용)
        self._error_px                = 0       # 좌우 픽셀 오차 (크랩 워킹용)
        self._horizontal_error        = 0.0     # 수평 기울기 오차 (yaw 정렬용)
        self._marker_top_y            = 0       # 수직 거리 정렬용
        self._last_calc_time          = 0.0     # 쓰로틀용 — 처리 시도 기준
        self._last_found_time         = 0.0     # target 마커 신선도 기준
        self._any_marker_found_time   = 0.0     # 임의 마커 신선도 기준 (yaw 정렬용)
        self._current_target_id       = None

        # ── QoS 설정 ──
        qos_img = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        # ── Pub/Sub 설정 ──
        self.create_subscription(Image,  "/image_raw", self._img_callback,      qos_img)
        self.create_subscription(Int32,  "/carry",     self._carry_callback,    qos_reliable)
        self.create_subscription(String, "/fork_cmd",  self._fork_cmd_callback, qos_reliable)

        self._pub_vel        = self.create_publisher(Twist,  "/cmd_vel",    qos_reliable)
        self._pub_carry_done = self.create_publisher(String, "/carry_done", qos_reliable)
        self._pub_fork_done  = self.create_publisher(String, "/fork_done",  qos_reliable)

        self.get_logger().info("✅ IntegratedForkliftNode 초기화 완료")

    # ──────────────────────────────────────────────
    # 이미지 콜백
    # ──────────────────────────────────────────────
    def _img_callback(self, msg: Image) -> None:
        if self._current_target_id is None:
            return

        now = time.time()

        # [수정 1] 쓰로틀: 처리 시도 기준 (CPU 보호용) — 감지 성공과 무관하게 갱신
        if now - self._last_calc_time < 0.05:   # 20fps 쓰로틀
            return
        self._last_calc_time = now  # 처리 시도 시각 갱신 (쓰로틀 전용)

        try:
            img      = self._bridge.imgmsg_to_cv2(msg, "bgr8")
            center_x = img.shape[1] // 2
            corners, ids, _ = self._detector.detectMarkers(img)

            if ids is not None:
                flat_ids = ids.flatten()

                # ── ① 수평 정렬용: 화면 중앙에 가장 가까운 마커 (ID 무관) ──
                any_candidates = []
                for i, m_id in enumerate(flat_ids):
                    marker_cx = int(np.mean(corners[i][0][:, 0]))
                    any_candidates.append((abs(marker_cx - center_x), i, marker_cx))

                if any_candidates:
                    any_candidates.sort(key=lambda x: x[0])
                    _, yaw_i, _ = any_candidates[0]
                    yaw_mc = corners[yaw_i][0]
                    # 수평 기울기: 하단 오른쪽(2) - 하단 왼쪽(3) y좌표 차이
                    self._horizontal_error      = float(yaw_mc[2][1] - yaw_mc[3][1])
                    self._any_marker_found      = True
                    self._any_marker_found_time = now  # ID 무관 신선도 갱신

                # ── ② 크랩 워킹 / 거리 정렬용: target_id 마커만 사용 ──
                target_candidates = []
                for i, m_id in enumerate(flat_ids):
                    if m_id == self._current_target_id:
                        marker_cx = int(np.mean(corners[i][0][:, 0]))
                        target_candidates.append((abs(marker_cx - center_x), i, marker_cx))

                if target_candidates:
                    target_candidates.sort(key=lambda x: x[0])
                    _, best_i, best_cx = target_candidates[0]
                    mc = corners[best_i][0]
                    self._marker_top_y    = np.min(mc[:, 1])
                    self._error_px        = best_cx - center_x
                    self._marker_found    = True
                    self._last_found_time = now  # [수정 1] 감지 성공 시에만 갱신
                    return

                # target_id 마커는 없지만 다른 마커는 있는 경우
                # → yaw 정렬용 horizontal_error는 갱신됐으나 found는 False
                self._marker_found = False
                return

            # 마커 전혀 없음: 두 플래그 모두 False
            self._marker_found     = False
            self._any_marker_found = False

        except Exception as e:
            self.get_logger().error(f"Image callback error: {e}")

    # ──────────────────────────────────────────────
    # 트리거 콜백
    # ──────────────────────────────────────────────
    def _carry_callback(self, msg: Int32) -> None:
        tid = msg.data
        if tid not in self.ALLOWED_IDS:
            self.get_logger().error(f"❌ Invalid ID: {tid}")
            return

        if self._is_running:
            self.get_logger().warn("⚠️ Already running a mission.")
            return

        self._is_running        = True
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

    # ──────────────────────────────────────────────
    # 유틸리티
    # ──────────────────────────────────────────────
    def _stop(self) -> None:
        stop_twist = Twist()
        deadline = time.time() + 0.3
        while time.time() < deadline:
            self._pub_vel.publish(stop_twist)
            time.sleep(0.02)

    def _wait_fresh(self, timeout: float = 5.0):
        start_wait = time.time()
        while (time.time() - start_wait) < timeout:
            if self._marker_found and \
               (time.time() - self._last_found_time < self.FRESH_THRESHOLD):
                return self._error_px * self.P2M
            time.sleep(0.05)
        return None

    def _timed_move(self, linear: float, angular: float, duration: float) -> None:
        twist = Twist()
        twist.linear.x  = float(linear)
        twist.angular.z = float(angular)

        end = time.time() + duration

        while rclpy.ok() and self._is_running:
            now = time.time()
            if now >= end:
                break
            remaining = end - now
            sleep_t   = min(0.02, remaining)
            self._pub_vel.publish(twist)
            time.sleep(sleep_t)

        self._stop()

    def _publish_fork_done(self, result: str) -> None:
        msg = String(); msg.data = result
        self._pub_fork_done.publish(msg)

    def _publish_carry_done(self, result: str) -> None:
        msg = String(); msg.data = result
        self._pub_carry_done.publish(msg)

    # ──────────────────────────────────────────────
    # 섹터 탐색
    # ──────────────────────────────────────────────
    def _search_sector(self, step_label: str) -> bool:
        self.get_logger().info(f"🔍 {step_label}: 마커 미감지 — 섹터 탐색 시작")

        t_rot = self.TURN_CONST / self.ANG_SPD

        self._timed_move(0.0, -self.ANG_SPD, t_rot)
        time.sleep(1.0)
        self._timed_move(self.MIS_LIN_SPD, 0.0, self.SECTOR_FWD_TIME)
        time.sleep(1.0)
        self._timed_move(0.0, self.ANG_SPD, t_rot)
        time.sleep(1.0)

        self.get_logger().info(f"🔍 {step_label}: 섹터 이동 완료 — 마커 재감지 대기 (5초)")
        redetect_start = time.time()
        while (time.time() - redetect_start) < 5.0:
            if self._marker_found and \
               (time.time() - self._last_found_time < self.FRESH_THRESHOLD):
                self.get_logger().info(f"✅ {step_label}: 섹터 탐색 후 마커 재감지 성공")
                return True
            time.sleep(0.1)

        self.get_logger().error(f"❌ {step_label}: 섹터 탐색 후에도 마커 미감지 — 미션 중단")
        return False

    # ──────────────────────────────────────────────
    # 수평 정렬 (Yaw Alignment)
    # ── _horizontal_error는 ID 무관, 가장 중앙 마커 기준 ──
    # ──────────────────────────────────────────────
    def _align_yaw(self, step_label: str) -> bool:
        """
        마커 하단 두 꼭짓점의 y좌표 차(horizontal_error)로 수평 정렬.
        _horizontal_error / _any_marker_found 는 ID 무관, 중앙에 가장 가까운 마커 기준.

        - 임의 마커 2초 유실 시 False 반환 → 호출부에서 섹터 탐색 처리
        - FRESH_THRESHOLD(0.15초) 이내 프레임만 정렬에 사용
        - 단일 저속(YAW_LIN_SPD) 고정
        - 종료 조건: abs(horizontal_error) < YAW_DEADZONE(0.8px)
        """
        self.get_logger().info(f"🔄 {step_label}: 수평 정렬 시작 (ID 무관, 중앙 마커 기준)")
        last_seen = self._any_marker_found_time if self._any_marker_found_time > 0.0 else time.time()

        while rclpy.ok() and self._is_running:

            if self._any_marker_found_time > last_seen:
                last_seen = self._any_marker_found_time

            age = time.time() - last_seen

            # 2초 유실 → False 반환 (호출부에서 섹터 탐색)
            if age > 2.0:
                self._stop()
                self.get_logger().warn(f"⚠️ {step_label}: 마커 {age:.1f}초 미갱신 — 정렬 실패")
                return False

            # 신선도 미달 → 정지 대기
            if age > self.FRESH_THRESHOLD:
                self._pub_vel.publish(Twist())
                time.sleep(0.05)
                continue

            err = self._horizontal_error

            if abs(err) < self.YAW_DEADZONE:
                self.get_logger().info(f"✅ {step_label}: 수평 정렬 완료 (오차: {err:.2f}px)")
                self._stop()
                return True

            twist = Twist()
            twist.angular.z = -self.YAW_LIN_SPD if err > 0 else self.YAW_LIN_SPD
            self._pub_vel.publish(twist)
            time.sleep(0.05)

        return False

    # ──────────────────────────────────────────────
    # Step 0-1: target 마커 최초 발견 대기 (10초 타임아웃)
    # ──────────────────────────────────────────────
    def _wait_for_marker(self) -> bool:
        self.get_logger().info("🔍 Step 0-1: Waiting for Marker (10s limit)...")
        search_start = time.time()

        while rclpy.ok() and self._is_running:
            if time.time() - search_start > 10.0:
                self.get_logger().error("❌ Step 0-1 Fail: Marker not found in 10s.")
                return False

            if self._marker_found and \
               (time.time() - self._last_found_time < self.FRESH_THRESHOLD):
                self.get_logger().info("✅ Step 0-1: Marker detected!")
                return True
            time.sleep(0.1)

        return False

    # ──────────────────────────────────────────────
    # Step 0-2: 수직 거리 정렬 (무제한)
    # 반환값:
    #   True  — 정렬 성공
    #   False — 노드 종료 등 정상 중단
    #   None  — 마커 2초 유실 → 섹터 탐색 필요
    # ──────────────────────────────────────────────
    def _do_zeroing(self, target_y: int):
        self.get_logger().info(
            f"🔍 Step 0-2: Zeroing 시작 — "
            f"is_running={self._is_running} "
            f"last_found={self._last_found_time:.3f}")
        last_seen     = self._last_found_time if self._last_found_time > 0.0 else time.time()
        last_log_time = 0.0

        while rclpy.ok() and self._is_running:

            if self._last_found_time > last_seen:
                last_seen = self._last_found_time

            age = time.time() - last_seen
            now = time.time()

            # 0.5초마다 상태 로그
            if now - last_log_time > 0.5:
                self.get_logger().info(
                    f"[0-2] marker_found={self._marker_found} "
                    f"age={age:.2f}s "
                    f"top_y={self._marker_top_y:.0f} "
                    f"target_y={target_y} "
                    f"diff={target_y - self._marker_top_y:.0f}px")
                last_log_time = now

            # 2초 미갱신 → 유실
            if age > 2.0:
                self._stop()
                self.get_logger().warn(
                    f"⚠️ Step 0-2: 마커 {age:.1f}초 미갱신 — 섹터 탐색 필요")
                return None

            # 신선도 미달 → 정지 대기
            if age > self.FRESH_THRESHOLD:
                self._pub_vel.publish(Twist())
                time.sleep(0.05)
                continue

            # 신선한 프레임
            diff = target_y - self._marker_top_y
            if abs(diff) <= 10:
                self.get_logger().info(f"📍 Zero-point reached: {self._marker_top_y:.1f}px")
                self._stop()
                return True

            twist = Twist()
            twist.linear.x = self.ZERO_LIN_SPD if diff > 0 else -self.ZERO_LIN_SPD
            self._pub_vel.publish(twist)
            time.sleep(0.05)

        return False

    # ──────────────────────────────────────────────
    # 메인 시퀀스
    # ──────────────────────────────────────────────
    def _run_full_sequence(self) -> None:
        self.get_logger().info(f"🔔 Mission Start! ID: {self._current_target_id}")

        try:
            self.get_logger().info("⏳ 카메라 프레임 안정화 대기 (5초)...")
            time.sleep(5.0)

            # ── Step 0-0: 수평 정렬 (ID 무관 임의 마커 기준) ──────────────────
            # 섹터 탐색은 전체 미션 통틀어 1회만 — 아래 두 조건 중 먼저 걸린 쪽에서 발동
            #   ① 수평 정렬 자체 실패 (아무 마커 없음)
            #   ② 수평 정렬 성공했지만 target ID 마커가 없음
            self.get_logger().info("🔄 Step 0-0: 수평 정렬 시작")
            need_sector = False

            if not self._align_yaw("Step 0-0"):
                self.get_logger().info("⚠️ Step 0-0: 수평 정렬 실패(마커 없음) — 섹터 탐색 예약")
                need_sector = True
            else:
                time.sleep(1.0)
                target_fresh = self._marker_found and \
                               (time.time() - self._last_found_time < self.FRESH_THRESHOLD)
                if not target_fresh:
                    self.get_logger().info("⚠️ Step 0-0: target 마커 미감지 — 섹터 탐색 예약")
                    need_sector = True

            if need_sector:
                if not self._search_sector("Step 0-0"):
                    raise RuntimeError("Step 0-0: 섹터 탐색 실패 — 마커 없음")
                time.sleep(1.0)
                if not self._align_yaw("Step 0-0 섹터 후 재정렬"):
                    raise RuntimeError("Step 0-0: 섹터 탐색 후 수평 정렬 실패")
                time.sleep(1.0)

            self.get_logger().info("🔍 Step 0-1 ~ 0-2: 마커 발견 + 수직 거리 정렬 진입")

            # ── Step 0-1: target 마커 발견 대기 ──
            if not self._wait_for_marker():
                # 타임아웃 — 섹터 탐색 잔여 있으면 실행
                if need_sector:
                    raise RuntimeError("Step 0-1: target 마커 미발견 (섹터 탐색 소진)")
                self.get_logger().info("⚠️ Step 0-1: target 마커 미발견 — 섹터 탐색 진입 (잔여 1회)")
                if not self._search_sector("Step 0-1"):
                    raise RuntimeError("Step 0-1: 섹터 탐색 실패 — 마커 없음")
                need_sector = True
                time.sleep(1.0)
                if not self._align_yaw("Step 0-1 섹터 후 재정렬"):
                    raise RuntimeError("Step 0-1: 섹터 탐색 후 수평 정렬 실패")
                time.sleep(1.0)
                if not self._wait_for_marker():
                    raise RuntimeError("Step 0-1: 섹터 탐색 후에도 target 마커 미발견")

            # ── Step 0-2 진입 전: 마커 신선도 재확인 ──
            target_fresh_02 = self._marker_found and \
                              (time.time() - self._last_found_time < self.FRESH_THRESHOLD)
            if not target_fresh_02:
                if need_sector:
                    raise RuntimeError("Step 0-2 진입 전: 마커 유실 (섹터 탐색 소진)")
                self.get_logger().info("⚠️ Step 0-2 진입 전: 마커 유실 — 섹터 탐색 진입 (잔여 1회)")
                if not self._search_sector("Step 0-2 pre"):
                    raise RuntimeError("Step 0-2 진입 전: 섹터 탐색 실패 — 마커 없음")
                need_sector = True
                time.sleep(1.0)
                if not self._align_yaw("Step 0-2 섹터 후 재정렬"):
                    raise RuntimeError("Step 0-2 진입 전: 섹터 탐색 후 수평 정렬 실패")
                time.sleep(1.0)
                if not self._wait_for_marker():
                    raise RuntimeError("Step 0-2 진입 전: 섹터 탐색 후에도 마커 미발견")

            # ── Step 0-2: 수직 거리 정렬 ──
            self.get_logger().info("🔍 Step 0-2: 수직 거리 정렬 진입")
            zeroing_result = self._do_zeroing(self.TARGET_Y)

            if zeroing_result is None:
                # 마커 유실 → 섹터 탐색 (미소진 시 1회)
                if need_sector:
                    raise RuntimeError("Step 0-2: 마커 유실 (섹터 탐색 소진)")
                self.get_logger().info("⚠️ Step 0-2: 마커 유실 — 섹터 탐색 진입 (잔여 1회)")
                if not self._search_sector("Step 0-2"):
                    raise RuntimeError("Step 0-2: 섹터 탐색 실패 — 마커 없음")
                need_sector = True
                time.sleep(1.0)
                if not self._align_yaw("Step 0-2 섹터 후 재정렬"):
                    raise RuntimeError("Step 0-2: 섹터 탐색 후 수평 정렬 실패")
                time.sleep(1.0)
                if not self._wait_for_marker():
                    raise RuntimeError("Step 0-2: 섹터 탐색 후 마커 미발견")
                # 섹터 소진 — 재시도 중 유실돼도 추가 탐색 없이 중단
                zeroing_result = self._do_zeroing(self.TARGET_Y)
                if not zeroing_result:
                    raise RuntimeError("Step 0-2: 섹터 탐색 후 수직 정렬 실패")
            elif zeroing_result is False:
                raise RuntimeError("Step 0-2: 수직 거리 정렬 실패")

            time.sleep(1.0)

            # ── Step 1-1: 수평 정렬 → (필요 시 섹터 탐색) → 크랩 워킹 1회차 ──
            # Step 0-0에서 섹터 탐색을 안 했고, 여기서 수평 정렬 실패 시 섹터 탐색 1회 실행
            self.get_logger().info("🎯 Step 1-1: 수평 정렬 후 크랩 워킹 1회차")
            if not self._align_yaw("Step 1-1 Yaw"):
                if need_sector:
                    # 이미 섹터 탐색을 했음 → 즉시 중단
                    raise RuntimeError("Step 1-1: 수평 정렬 실패 — 마커 없음 (섹터 탐색 소진)")
                # 섹터 탐색 미사용 → 1회 실행
                self.get_logger().info("⚠️ Step 1-1: 수평 정렬 실패 — 섹터 탐색 진입 (잔여 1회)")
                if not self._search_sector("Step 1-1"):
                    raise RuntimeError("Step 1-1: 섹터 탐색 실패 — 마커 없음")
                need_sector = True  # 소진 표시
                time.sleep(1.0)
                if not self._align_yaw("Step 1-1 섹터 후 재정렬"):
                    raise RuntimeError("Step 1-1: 섹터 탐색 후 수평 정렬 실패")

            time.sleep(1.0)

            err_m = self._wait_fresh(timeout=5.0)
            if err_m is None:
                self.get_logger().warn("Step 1-1: 마커 유실, 재시도...")
                time.sleep(0.5)
                err_m = self._wait_fresh(timeout=3.0)
                if err_m is None:
                    raise RuntimeError("Step 1-1: 마커 유실로 크랩 워킹 불가")

            if abs(err_m) >= self.DEADZONE_M:
                self.get_logger().info(f"   - Step 1-1 크랩 워킹: error {err_m:.4f}m")
                rot_dir = -1.0 if err_m > 0 else 1.0
                t_rot   = self.TURN_CONST / self.ANG_SPD
                t_side  = abs(err_m) / self.LIN_SPD

                self._timed_move(0.0, self.ANG_SPD * rot_dir,  t_rot)
                self._timed_move(self.LIN_SPD, 0.0,            t_side)
                self._timed_move(0.0, -self.ANG_SPD * rot_dir, t_rot)

                time.sleep(2.0)
            else:
                self.get_logger().info("✅ Step 1-1: 오차 데드존 이내, 크랩 워킹 생략")

            self.get_logger().info("🎯 Step 1-2: 크랩 워킹 2회차")

            if not self._align_yaw("Step 1-2 Yaw"):
                raise RuntimeError("Step 1-2: 수평 정렬 실패")

            time.sleep(1.0)

            err_m = self._wait_fresh(timeout=5.0)
            if err_m is None:
                self.get_logger().warn("Step 1-2: 마커 유실, 재시도...")
                time.sleep(0.5)
                err_m = self._wait_fresh(timeout=3.0)
                if err_m is None:
                    raise RuntimeError("Step 1-2: 마커 유실로 크랩 워킹 불가")

            if abs(err_m) >= self.DEADZONE_M:
                self.get_logger().info(f"   - Step 1-2 크랩 워킹: error {err_m:.4f}m")
                rot_dir = -1.0 if err_m > 0 else 1.0
                t_rot   = self.TURN_CONST / self.ANG_SPD
                t_side  = abs(err_m) / self.LIN_SPD

                self._timed_move(0.0, self.ANG_SPD * rot_dir,  t_rot)
                self._timed_move(self.LIN_SPD, 0.0,            t_side)
                self._timed_move(0.0, -self.ANG_SPD * rot_dir, t_rot)

                time.sleep(2.0)
            else:
                self.get_logger().info("✅ Step 1-2: 오차 데드존 이내, 크랩 워킹 생략")

            self.get_logger().info("🔄 Step 2 전: 수평 정렬")
            if not self._align_yaw("Step 2 pre-align"):
                raise RuntimeError("Step 2 전 수평 정렬 실패 — 마커 없음")

            time.sleep(1.0)

            self.get_logger().info("➡️ Step 2: Forwarding...")
            self._timed_move(self.MIS_LIN_SPD, 0.0, 7.0)

            self.get_logger().info("🔼 Step 3: Fork UP...")
            self._fork.fork_up()
            self._publish_fork_done("UP_DONE")

            self.get_logger().info("⬅️ Step 4: Backwarding...")
            self._timed_move(-self.MIS_LIN_SPD, 0.0, 8.0)

            self._publish_carry_done(f"success: ID {self._current_target_id}")
            self.get_logger().info("✨ Mission Complete!")

        except Exception as e:
            self.get_logger().error(f"❌ Mission Failed: {e}")
        finally:
            self._stop()
            self._is_running        = False
            self._current_target_id = None

    def destroy_node(self) -> None:
        if hasattr(self, '_ser') and self._ser.is_open:
            self._ser.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IntegratedForkliftNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()