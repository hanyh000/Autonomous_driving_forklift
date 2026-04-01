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
        self._is_running          = False
        self._marker_found        = False
        self._error_px            = 0      # 좌우 픽셀 오차 (크랩 워킹용)
        self._horizontal_error    = 0.0    # 수평 기울기 오차 (yaw 정렬용)
        self._marker_top_y        = 0      # 수직 거리 정렬용
        self._last_calc_time      = 0.0    # 쓰로틀용 — 처리 시도 기준
        self._last_found_time     = 0.0    # 신선도용 — 감지 성공 기준 [수정 1]
        self._current_target_id   = None

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
                # 같은 ID 마커가 여러 개일 때 화면 중앙에 가장 가까운 것 선택
                candidates = []
                for i, m_id in enumerate(ids.flatten()):
                    if m_id == self._current_target_id:
                        marker_cx = int(np.mean(corners[i][0][:, 0]))
                        candidates.append((abs(marker_cx - center_x), i, marker_cx))

                if candidates:
                    candidates.sort(key=lambda x: x[0])
                    _, best_i, best_cx = candidates[0]

                    mc = corners[best_i][0]
                    self._marker_top_y     = np.min(mc[:, 1])
                    self._error_px         = best_cx - center_x
                    # 수평 기울기: 하단 오른쪽(2) - 하단 왼쪽(3) y좌표 차이
                    # 0에 가까울수록 마커와 정면으로 정렬된 상태
                    self._horizontal_error = float(mc[2][1] - mc[3][1])
                    self._marker_found     = True
                    self._last_found_time  = now  # [수정 1] 감지 성공 시에만 갱신
                    return

            # 감지 실패: found만 False, last_found_time은 건드리지 않음 [수정 1]
            self._marker_found = False

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
        """
        정지 + 관성 안정화.
        [수정 4] 정지 명령을 0.3초간 반복 전송 — 랙으로 인한 1회 전송 유실 방지.
        """
        stop_twist = Twist()  # 전부 0
        deadline = time.time() + 0.3
        while time.time() < deadline:
            self._pub_vel.publish(stop_twist)
            time.sleep(0.02)

    def _wait_fresh(self, timeout: float = 5.0):
        """
        신선한 error_px 값이 올 때까지 블로킹.
        FRESH_THRESHOLD(0.15초) 이내 프레임만 신뢰 — 밀림 방지 핵심.
        """
        start_wait = time.time()
        while (time.time() - start_wait) < timeout:
            if self._marker_found and \
               (time.time() - self._last_found_time < self.FRESH_THRESHOLD):
                return self._error_px * self.P2M
            time.sleep(0.05)
        return None

    def _timed_move(self, linear: float, angular: float, duration: float) -> None:
        """
        [수정 3] 실제 경과 시간 기준 종료 + min(0.02, remaining) 슬립.
        - 루프 카운트 누적 오차 제거
        - 종료 직전 오버슈트 방지
        - 정지 명령 반복 전송(_stop)으로 확실한 정지
        """
        twist = Twist()
        twist.linear.x  = float(linear)
        twist.angular.z = float(angular)

        end = time.time() + duration

        while rclpy.ok() and self._is_running:
            now = time.time()
            if now >= end:
                break                            # 실제 경과 시간 기준 종료

            remaining = end - now
            sleep_t   = min(0.02, remaining)     # 남은 시간보다 더 자지 않음
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
        """
        마커 미감지 시 캠 시야각 한계를 보완하는 섹터 이동.
        60cm 섹터를 -30 / +30 으로 나눈 개념.

        우회전 90° → 20cm 전진(MIS_LIN_SPD, 4.0초) → 좌회전 90° 복귀
        → ★ 5초 대기 (마커 재감지 시도)
        → 감지됨 : True  반환 (정상 진행)
        → 미감지 : False 반환 (미션 중단 — 진짜 마커 없음)

        호출 위치: Step 0-0 수평 정렬 전, Step 1-1 수평 정렬 전
        """
        self.get_logger().info(f"🔍 {step_label}: 마커 미감지 — 섹터 탐색 시작")

        t_rot = self.TURN_CONST / self.ANG_SPD

        # 우회전 90°
        self._timed_move(0.0, -self.ANG_SPD, t_rot)
        # ★ 회전 후 대기 (밀림 방지)
        time.sleep(1.0)

        # 20cm 전진 (MIS_LIN_SPD 기준 4.0초)
        self._timed_move(self.MIS_LIN_SPD, 0.0, self.SECTOR_FWD_TIME)
        # ★ 전진 후 대기 (밀림 방지)
        time.sleep(1.0)

        # 좌회전 90° 복귀
        self._timed_move(0.0, self.ANG_SPD, t_rot)
        # ★ 복귀 후 대기 (밀림 방지)
        time.sleep(1.0)

        # 5초 대기하며 마커 재감지 시도
        self.get_logger().info(f"🔍 {step_label}: 섹터 이동 완료 — 마커 재감지 대기 (5초)")
        redetect_start = time.time()
        while (time.time() - redetect_start) < 5.0:
            if self._marker_found and \
               (time.time() - self._last_found_time < self.FRESH_THRESHOLD):
                self.get_logger().info(f"✅ {step_label}: 섹터 탐색 후 마커 재감지 성공")
                return True
            time.sleep(0.1)

        # 5초 후에도 미감지 → 진짜 마커 없음
        self.get_logger().error(f"❌ {step_label}: 섹터 탐색 후에도 마커 미감지 — 미션 중단")
        return False

    # ──────────────────────────────────────────────
    # 수평 정렬 (Yaw Alignment)
    # ──────────────────────────────────────────────
    def _align_yaw(self, step_label: str) -> bool:
        """
        마커 하단 두 꼭짓점의 y좌표 차(horizontal_error)로 수평 정렬.

        - 마커 유실 2초 경과 시 강제 정지 후 무한 대기
        - [수정 2] FRESH_THRESHOLD(0.15초) 이내 프레임만 정렬에 사용
        - 단일 저속(YAW_LIN_SPD) 고정 — 오버슈트 방지
        - 루프 주기 0.05초
        - 종료 조건: abs(horizontal_error) < YAW_DEADZONE(0.8px)
        """
        self.get_logger().info(f"🔄 {step_label}: 수평 정렬 시작")

        while rclpy.ok() and self._is_running:

            # ★ 마커 유실 2초 경과 시 강제 정지 후 무한 대기 (밀림 방지)
            if self._last_found_time > 0.0 and \
               (time.time() - self._last_found_time > 2.0):
                self._pub_vel.publish(Twist())
                time.sleep(0.1)
                continue

            if not self._marker_found:
                time.sleep(0.05)
                continue

            # [수정 2] 신선도 체크: 0.15초 이내 프레임만 사용
            if time.time() - self._last_found_time > self.FRESH_THRESHOLD:
                time.sleep(0.05)
                continue

            err = self._horizontal_error

            if abs(err) < self.YAW_DEADZONE:
                self.get_logger().info(f"✅ {step_label}: 수평 정렬 완료 (오차: {err:.2f}px)")
                self._stop()
                return True

            # 단일 저속 고정
            twist = Twist()
            twist.angular.z = -self.YAW_LIN_SPD if err > 0 else self.YAW_LIN_SPD
            self._pub_vel.publish(twist)
            time.sleep(0.05)

        return False

    # ──────────────────────────────────────────────
    # Step 0-1 ~ 0-2: 마커 발견 및 수직 거리 정렬
    # ──────────────────────────────────────────────
    def _find_zero_point(self, target_y: int) -> bool:
        """
        Step 0-1: 마커 최초 발견 대기 (10초 타임아웃)
        Step 0-2: 수직 거리 정렬 - marker_top_y → TARGET_Y ±10px (무제한)
        [수정 2] 정렬 루프에 FRESH_THRESHOLD 신선도 체크 추가
        """
        self.get_logger().info("🔍 Step 0-1: Waiting for Marker (10s limit)...")
        search_start    = time.time()
        found_initially = False

        # [0-1] 마커 최초 발견 대기
        while rclpy.ok() and self._is_running:
            if time.time() - search_start > 10.0:
                self.get_logger().error("❌ Step 0-1 Fail: Marker not found in 10s.")
                return False

            if self._marker_found and \
               (time.time() - self._last_found_time < self.FRESH_THRESHOLD):
                self.get_logger().info("✅ Marker detected! Starting zeroing...")
                found_initially = True
                break
            time.sleep(0.1)

        # [0-2] 수직 거리 정렬 (무제한)
        if found_initially:
            self.get_logger().info("🔍 Step 0-2: Zeroing (No Timeout)...")
            while rclpy.ok() and self._is_running:

                # ★ 마커 유실 2초 경과 시 강제 정지 후 대기 (밀림 방지)
                if time.time() - self._last_found_time > 2.0:
                    self._pub_vel.publish(Twist())
                    time.sleep(0.1)
                    continue

                if not self._marker_found:
                    time.sleep(0.05)
                    continue

                # [수정 2] 신선도 체크: 0.15초 이내 프레임만 사용
                if time.time() - self._last_found_time > self.FRESH_THRESHOLD:
                    time.sleep(0.05)
                    continue

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

            # ── Step 0-0: [섹터 탐색] → 수평 정렬 ──────────
            marker_fresh_00 = self._marker_found and \
                              self._last_found_time > 0.0 and \
                              (time.time() - self._last_found_time < self.FRESH_THRESHOLD)
            if not marker_fresh_00:
                self.get_logger().info("⚠️ Step 0-0: 마커 미감지 — 섹터 탐색 진입")
                if not self._search_sector("Step 0-0"):
                    raise RuntimeError("Step 0-0: 섹터 탐색 실패 — 마커 없음")

            # ★ 섹터 탐색(또는 마커 감지 확인) 후 → 수평 정렬 전 대기 (밀림 방지)
            time.sleep(1.0)

            if not self._align_yaw("Step 0-0"):
                raise RuntimeError("Step 0-0: 수평 정렬 실패")

            # ★ 수평 정렬 후 → 마커 탐색 전 대기 (밀림 방지)
            time.sleep(1.0)

            # ── Step 0-1 ~ 0-2: 마커 발견 + 수직 거리 정렬 ──
            self.get_logger().info("🔍 Step 0-1 ~ 0-2: 마커 발견 + 수직 거리 정렬 진입")
            if not self._find_zero_point(self.TARGET_Y):
                raise RuntimeError("Step 0-1 ~ 0-2: 마커 발견/거리 정렬 실패")

            # ★ 수직 정렬 완료 후 → Step 1 진입 전 대기 (밀림 방지)
            time.sleep(1.0)

            # ── Step 1-1: [섹터 탐색] → 수평 정렬 → 크랩 워킹 1회차 ──
            self.get_logger().info("🎯 Step 1-1: 크랩 워킹 1회차")

            marker_fresh_11 = self._marker_found and \
                              self._last_found_time > 0.0 and \
                              (time.time() - self._last_found_time < self.FRESH_THRESHOLD)
            if not marker_fresh_11:
                self.get_logger().info("⚠️ Step 1-1: 마커 미감지 — 섹터 탐색 진입")
                if not self._search_sector("Step 1-1"):
                    raise RuntimeError("Step 1-1: 섹터 탐색 실패 — 마커 없음")

            # ★ 섹터 탐색(또는 마커 감지 확인) 후 → 수평 정렬 전 대기 (밀림 방지)
            time.sleep(1.0)

            if not self._align_yaw("Step 1-1 Yaw"):
                raise RuntimeError("Step 1-1: 수평 정렬 실패")

            # ★ 수평 정렬 후 → 크랩 워킹 판단 전 대기 (밀림 방지)
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

                # ★ 크랩 워킹 후 대기 — 쌓인 구버퍼 flush (밀림 방지 핵심)
                time.sleep(2.0)
            else:
                self.get_logger().info("✅ Step 1-1: 오차 데드존 이내, 크랩 워킹 생략")

            # ── Step 1-2: 수평 정렬 → 크랩 워킹 2회차 ────
            self.get_logger().info("🎯 Step 1-2: 크랩 워킹 2회차")

            if not self._align_yaw("Step 1-2 Yaw"):
                raise RuntimeError("Step 1-2: 수평 정렬 실패")

            # ★ 수평 정렬 후 → 크랩 워킹 판단 전 대기 (밀림 방지)
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

                # ★ 크랩 워킹 후 대기 — 쌓인 구버퍼 flush (밀림 방지 핵심)
                time.sleep(2.0)
            else:
                self.get_logger().info("✅ Step 1-2: 오차 데드존 이내, 크랩 워킹 생략")

            # ── Step 2: 전진 ─────────────────────────────
            self.get_logger().info("➡️ Step 2: Forwarding...")
            self._timed_move(self.MIS_LIN_SPD, 0.0, 7.0)

            # ── Step 3: 포크 UP ──────────────────────────
            self.get_logger().info("🔼 Step 3: Fork UP...")
            self._fork.fork_up()
            self._publish_fork_done("UP_DONE")

            # ── Step 4: 후진 ─────────────────────────────
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
