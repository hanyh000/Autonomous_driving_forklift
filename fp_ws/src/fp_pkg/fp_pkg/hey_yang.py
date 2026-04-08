#!/usr/bin/env python3
"""
FSM Align Line-Tracing Unload Node  v7.2
TurtleBot3 Waffle Pi + Forklift Add-on
ROS2 Humble | Remote PC Control

■ v7.2 변경 요약 (v7.0 → v7.2)
  ──────────────────────────────────────────────────────────────────────────
  v7.1 에서 시도한 기능(A1~A6) 중 현장 환경에서 신뢰성 있게 동작하지 않는
  항목(A1 직진, A2 tail 감지, A6 ReturnStraightState)을 제거하고,
  독립적으로 유효한 항목만 v7.0 위에 최소 침습적으로 적용한다.

  [B1]  _cb_unload 검증 기준 수정 — COLOR_HSV → COLOR_TO_MARKER_ID
        v7.1 에서 COLOR_HSV 에 'BLUE_RETURN' 키를 추가함으로써
        RED·YELLOW 수신 시 'RED'/'YELLOW' not in COLOR_HSV 가
        False(통과)로 남아 있어야 하지만, 향후 키 구조 변경에도 안전하도록
        검증 기준을 COLOR_TO_MARKER_ID 로 명시 변경.

  [B2]  BLUE 복귀 라인트레이싱 HSV 분리 — 오인식 차단 + 복귀 감지 양립
        실측 근거 (rqt 이미지 HSV 분석):
          · 파란 테이프   HSV: H=110~127, S=53~184, V=73~160
          · 사물함 나사   HSV: H=125~130, S=56~60,  V=98~104  (오인식 원인)
          · 어두운 회색 배경 반사: H=100~130, S<50, V 다양      (오인식 원인)

        적용 전략 — 3중 방어:
          방어 1 (접근 returning=False): COLOR_HSV['BLUE'] S≥80, V≥80
            → 사물함 나사(S≤60) 완전 차단, 기존 v7.0 과 동일
          방어 2 (복귀 returning=True):  COLOR_HSV['BLUE_RETURN'] S≥50, V≥60
            → 조명 측·후면 조사로 인한 채도 저하(S≈50) 대응
            → MORPH_OPEN 커널 3×3 → 5×5 (소면적 잡음 억제 강화)
            → m00 임계값 125 → 300 (실제 라인 m00≈145000, 잡음 m00<300)

        구현: _detect_line() 내부에서 returning + target_color 분기만 추가.
        RED·YELLOW 동작에 영향 없음.

  [B3]  ROI 비율 동적 확장 — 복귀 구간 라인 가시성 향상
        · 기존: ROI_RATIO=0.40 (하단 40% 고정)
        · returning=True 시: ROI_RATIO_RETURN=0.60 (하단 60%)
        · _process_frame() 에서 returning 플래그로 자동 선택.
        · 사물함 나사(원본 y=16~44px)는 ROI 60% 기준 상단 경계(y=240px)
          보다 훨씬 위에 있으므로 ROI 확대 후에도 나사가 ROI 내로 진입하지 않음.

■ v7.1 에서 제거된 항목 (현장 신뢰성 부족)
  · A1  _s_return_straight()    — tail 감지에 의존, 단독 동작 불안정
  · A2  _detect_line_tail()     — 광택 바닥·회색 배경에서 끝단 오판정 빈발
  · A3  APPROACH armed 분기     — A2 제거에 따라 불필요
  · A6  ReturnStraightState     — A1 제거에 따라 불필요

■ v7.0 기능 전체 보존
  · [R1]~[R10] 리팩토링 결과 모두 유지.
  · 핵심 우선순위 원칙, 마커 ID 등록 규칙, 상태 목록 변경 없음.
  · 귀환 라인 소실 → DONE 동작 v7.0 과 동일.
"""

# ════════════════════════════════════════════════════════════════════════════
#  임포트
# ════════════════════════════════════════════════════════════════════════════
import math
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import String

# ════════════════════════════════════════════════════════════════════════════
#  모듈 레벨 상수
# ════════════════════════════════════════════════════════════════════════════

def _make_qos(reliability, durability, depth: int,
              history=HistoryPolicy.KEEP_LAST) -> QoSProfile:
    return QoSProfile(
        reliability=reliability,
        durability=durability,
        history=history,
        depth=depth,
    )

_QOS_RELIABLE = _make_qos(ReliabilityPolicy.RELIABLE,
                           DurabilityPolicy.VOLATILE, depth=1)
_QOS_CMD_VEL  = _make_qos(ReliabilityPolicy.RELIABLE,
                           DurabilityPolicy.VOLATILE, depth=10)
_QOS_IMG      = _make_qos(ReliabilityPolicy.BEST_EFFORT,
                           DurabilityPolicy.VOLATILE, depth=1)

# [B2] BLUE 를 접근용(BLUE)·복귀용(BLUE_RETURN) 으로 분리.
#      RED·YELLOW 는 v7.0 과 동일.
#
#  실측 파란 테이프   HSV: H=110~127, S=53~184, V=73~160
#  사물함 나사 도트   HSV: H=125~130, S=56~60,  V=98~104  → S≥80 으로 차단
#  어두운 회색 배경   HSV: H=100~130, S<50                → S≥80 으로 차단
#
COLOR_HSV: dict = {
    'BLUE':        [((100, 80,  80), (130, 255, 255))],   # 접근: S≥80 엄격 (v7.0 동일)
    'BLUE_RETURN': [((100, 50,  60), (130, 255, 255))],   # 복귀: S≥50, V≥60 완화
    'RED':         [((0,   80,  80), (10,  255, 255)),
                    ((170, 80,  80), (180, 255, 255))],
    'YELLOW':      [((14,  95, 130), (24,  255, 255))],
}

# [B1] 수신 색상 검증 기준 — COLOR_TO_MARKER_ID 키 집합 사용
#      (COLOR_HSV 에 'BLUE_RETURN' 가 추가되어도 검증 로직이 영향받지 않음)
COLOR_TO_MARKER_ID: dict = {'BLUE': 0, 'RED': 1, 'YELLOW': 2}


# ════════════════════════════════════════════════════════════════════════════
#  FSM 상태 레이블
# ════════════════════════════════════════════════════════════════════════════
class St:
    WAIT           = 'WAIT'
    IDLE           = 'IDLE'
    SEARCH         = 'SEARCH'
    APPROACH       = 'APPROACH'
    APPROACH_ALIGN = 'APPROACH_ALIGN'
    EMERGENCY_STOP = 'EMERGENCY_STOP'
    UNLOAD_ALIGN   = 'UNLOAD_ALIGN'
    UNLOAD_STANDBY = 'UNLOAD_STANDBY'
    RETURN         = 'RETURN'
    DONE           = 'DONE'


# ════════════════════════════════════════════════════════════════════════════
#  상태 데이터 클래스 (v7.0 [R1] 유지)
# ════════════════════════════════════════════════════════════════════════════

@dataclass
class MarkerState:
    """ArUco 마커 인식 결과."""
    found:  bool                           = False
    id:     Optional[int]                  = None
    top_y:  float                          = 0.0
    bot_y:  float                          = 0.0
    cx:     int                            = 0
    tl:     Optional[Tuple[float, float]]  = None
    tr:     Optional[Tuple[float, float]]  = None


@dataclass
class LidarState:
    """LiDAR 처리 결과."""
    front_dist:      float = float('inf')
    narrow_dist:     float = float('inf')
    obstacle_front:  bool  = False
    pre_estop_state: str   = St.APPROACH


@dataclass
class OdomState:
    """오도메트리 원시값."""
    x:  float = 0.0
    y:  float = 0.0
    oz: float = 0.0
    ow: float = 1.0


@dataclass
class SlipState:
    """슬립 계측 결과 저장소."""
    turn_fallback:    float = 2.0
    turn_correction:  float = 2.0
    turn_measured:    bool  = False

    fwd_measured: bool  = False
    fwd_t:        float = 0.0
    fwd_d:        float = 0.0
    fwd_ratio:    float = 0.0


# ════════════════════════════════════════════════════════════════════════════
#  메인 노드
# ════════════════════════════════════════════════════════════════════════════
class FSMUnloadNode(Node):

    def __init__(self) -> None:
        super().__init__('fsm_unload_node')
        self._lock = threading.Lock()
        self._init_params()
        self._init_state()
        self._init_ros()
        threading.Thread(target=self._fsm_loop, daemon=True).start()
        self.get_logger().info(
            f"✅ FSMUnloadNode v7.2 준비 완료 — WAIT 대기 중\n"
            f"   이미지 토픽  : /image_raw/compressed (15 fps)\n"
            f"   ArUco 처리 주기: {self.ARUCO_PROCESS_EVERY} 프레임마다 1회\n"
            f"   [회전] turn_correction={self._slip.turn_correction:.4f}  "
            f"turn_measured={self._slip.turn_measured}\n"
            f"   [전진] fwd_measured={self._slip.fwd_measured}\n"
            f"   [B2] BLUE 복귀 HSV: S≥50, V≥60  커널=5×5  m00≥300\n"
            f"   [B3] ROI 복귀 확장: {self.ROI_RATIO_RETURN*100:.0f}%"
        )

    # ── 튜닝 상수 ────────────────────────────────────────────────────────────
    def _init_params(self) -> None:
        # 속도
        self.SPD_LINE   = 0.05
        self.SPD_ALIGN  = 0.01
        self.SPD_ANG    = 0.40
        self.SPD_SEARCH = 0.40
        self.SPD_BACK   = 0.10

        # 카메라 / 마커
        self.FRAME_W            = 640
        self.FRAME_H            = 480
        self.ROI_RATIO          = 0.40     # 접근 시 하단 40% (v7.0 동일)
        self.ROI_RATIO_RETURN   = 0.60     # [B3] 복귀 시 하단 60%
        self.MARKER_STOP_BOT_Y  = 336
        self.MARKER_TOO_CLOSE_Y = 470
        self.ALIGN_CX_MIN       = 315
        self.ALIGN_CX_MAX       = 325
        self.ALIGN_TOP_TOL      = 5.0

        # 프레임 스킵
        self.CB_SKIP_EVERY       = 1
        self.ARUCO_PROCESS_EVERY = 3

        # LiDAR
        self.LIDAR_DIST            = 0.35
        self.LIDAR_ANGLE           = 30
        self.LIDAR_NARROW_ANGLE    = 5
        self.LIDAR_MEDIAN_N        = 5
        self.UNLOAD_TARGET_DIST    = 0.32
        self.LIDAR_UNLOAD_STOP     = 0.3
        self.LIDAR_UNLOAD_MIN_ODOM = 0.005

        # 회전 슬립 파라미터
        self.TURN_MIN_DEG            = 90.0
        self.TURN_SUPPLEMENT_DEG     = 45.0
        self.TURN_SUPPLEMENT_MIN_DEG = 10.0
        self.TURN_SLIP_TOL_DEG       = 5.0

        # odom yaw 안정화
        self.ODOM_STABLE_TOL_DEG = 1.0
        self.ODOM_STABLE_TIMEOUT = 5.0

        # RETURN 교차 검증
        self.RETURN_CORRECT_TOL_DEG = 5.0
        self.RETURN_TRIM_TIMEOUT    = 15.0

        # ── [B2] BLUE 복귀 전용 라인 감지 파라미터 ───────────────────────────
        # 실제 라인 m00 ≈ 145,000 (0.5배율 ROI 기준)
        # 잡음(도트·반사) m00 < 300 → 임계값 300 으로 분리
        self.LINE_M00_APPROACH   = 125   # 접근 (v7.0 동일)
        self.LINE_M00_RETURN     = 300   # 복귀 BLUE 전용
        self.LINE_MORPH_APPROACH = 3     # 접근 MORPH 커널 (v7.0 동일)
        self.LINE_MORPH_RETURN   = 5     # 복귀 MORPH 커널

    # ── 런타임 상태 ──────────────────────────────────────────────────────────
    def _init_state(self) -> None:
        self.state         = St.WAIT
        self.returning     = False
        self.target_color: Optional[str] = None
        self.endpoint_ids  = {100}

        self.line_detected = False
        self.line_cx: Optional[int] = None

        self._marker = MarkerState()
        self._lidar  = LidarState()
        self._odom   = OdomState()
        self._slip   = SlipState()

        self.fork_done = False

        self._cb_skip_counter    = 0
        self._aruco_skip_counter = 0

        self._last_corners_all: list = []
        self._last_ids_all           = None

        self._ignore_camera_event = threading.Event()
        self._frame_event         = threading.Event()

        self.aruco = _ARUCO_DETECTOR

    # ── ROS2 I/O ─────────────────────────────────────────────────────────────
    def _init_ros(self) -> None:
        self.sub_unload    = self.create_subscription(String,          '/get_unload',           self._cb_unload,    _QOS_RELIABLE)
        self.sub_img       = self.create_subscription(CompressedImage, '/image_raw/compressed', self._cb_image,     _QOS_IMG)
        self.sub_lidar     = self.create_subscription(LaserScan,       '/scan',                 self._cb_lidar,     qos_profile_sensor_data)
        self.sub_fork_done = self.create_subscription(String,          '/fork_done',            self._cb_fork_done, _QOS_RELIABLE)
        self.sub_odom      = self.create_subscription(Odometry,        '/odom',                 self._cb_odom,      qos_profile_sensor_data)

        self.pub_vel  = self.create_publisher(Twist,  '/cmd_vel',  _QOS_CMD_VEL)
        self.pub_fork = self.create_publisher(String, '/fork_cmd', _QOS_RELIABLE)
        self.pub_done = self.create_publisher(String, '/done',     _QOS_RELIABLE)

    # ════════════════════════════════════════════════════════════════════════
    #  ROS 콜백
    # ════════════════════════════════════════════════════════════════════════

    def _cb_unload(self, msg: String) -> None:
        color = msg.data.strip().upper()
        # [B1] 검증 기준: COLOR_TO_MARKER_ID (COLOR_HSV 키 구조와 독립)
        if color not in COLOR_TO_MARKER_ID:
            self.get_logger().warn(f"⚠️  알 수 없는 색상: '{color}'")
            return
        if self.state != St.WAIT:
            self.get_logger().warn("⚠️  미션 진행 중 — /get_unload 무시")
            return
        mid = COLOR_TO_MARKER_ID[color]
        with self._lock:
            self.target_color = color
            self.endpoint_ids = {100, mid}
            self.returning    = False
        self.get_logger().info(f"📩 /get_unload: {color} → 마커 ID {mid} → SEARCH")
        self._change_state(St.SEARCH)

    def _cb_image(self, msg: CompressedImage) -> None:
        if self._ignore_camera_event.is_set():
            return

        self._cb_skip_counter += 1
        if self._cb_skip_counter % self.CB_SKIP_EVERY != 0:
            return

        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is None:
                self.get_logger().warn("⚠️  _cb_image: imdecode 실패 (None)")
                return

            with self._lock:
                self.FRAME_H, self.FRAME_W = img.shape[:2]

            self._aruco_skip_counter += 1
            do_aruco = (self._aruco_skip_counter % self.ARUCO_PROCESS_EVERY == 0)
            self._process_frame(img, do_aruco=do_aruco)

        except Exception as exc:
            self.get_logger().error(f"_cb_image 오류: {exc}")

    def _cb_lidar(self, msg: LaserScan) -> None:
        try:
            valid_wide   = self._lidar_valid_ranges(msg, self.LIDAR_ANGLE)
            valid_narrow = self._lidar_valid_ranges(msg, self.LIDAR_NARROW_ANGLE)

            front_dist  = min(valid_wide)   if valid_wide   else float('inf')
            narrow_dist = min(valid_narrow) if valid_narrow else float('inf')
            obstacle    = bool(valid_wide and front_dist < self.LIDAR_DIST)

            with self._lock:
                self._lidar.front_dist  = front_dist
                self._lidar.narrow_dist = narrow_dist

            if obstacle and not self._lidar.obstacle_front:
                if self.state in (St.APPROACH, St.APPROACH_ALIGN):
                    with self._lock:
                        self._lidar.obstacle_front  = True
                        self._lidar.pre_estop_state = self.state
                    self._change_state(St.EMERGENCY_STOP)
            elif not obstacle and self._lidar.obstacle_front:
                with self._lock:
                    self._lidar.obstacle_front = False

        except Exception as exc:
            self.get_logger().error(f"_cb_lidar 오류: {exc}")

    def _cb_fork_done(self, msg: String) -> None:
        self.get_logger().info(f"🔔 /fork_done 수신: '{msg.data}'")
        with self._lock:
            self.fork_done = True

    def _cb_odom(self, msg: Odometry) -> None:
        with self._lock:
            self._odom.x  = msg.pose.pose.position.x
            self._odom.y  = msg.pose.pose.position.y
            self._odom.oz = msg.pose.pose.orientation.z
            self._odom.ow = msg.pose.pose.orientation.w

    # ════════════════════════════════════════════════════════════════════════
    #  이미지 처리
    # ════════════════════════════════════════════════════════════════════════

    def _process_frame(self, img: np.ndarray, do_aruco: bool = True) -> None:
        h, w = img.shape[:2]

        # [B3] returning 여부에 따라 ROI 비율 동적 선택
        with self._lock:
            returning = self.returning
        roi_ratio = self.ROI_RATIO_RETURN if returning else self.ROI_RATIO
        roi_top   = int(h * (1.0 - roi_ratio))

        if do_aruco:
            if self.state == St.UNLOAD_ALIGN:
                self._detect_aruco(img, roi_top_offset=0)
            else:
                self._detect_aruco(img[roi_top:h, :], roi_top_offset=roi_top)
            self._frame_event.set()

        self._detect_line(img[roi_top:h, :], w)

    def _detect_aruco(self, roi: np.ndarray, roi_top_offset: int = 0) -> None:
        """엔드포인트 ID에 해당하는 첫 번째 마커만 처리."""
        corners, ids, _ = self.aruco.detectMarkers(roi)
        self._last_corners_all = corners
        self._last_ids_all     = ids

        found = False
        if ids is not None:
            for i, mid in enumerate(ids.flatten().tolist()):
                if int(mid) not in self.endpoint_ids:
                    continue

                pts            = corners[i][0]
                tl, tr, br, bl = pts
                h_roi          = roi.shape[0]

                def to_orig(pt: np.ndarray) -> Tuple[float, float]:
                    return float(pt[0]), float(pt[1]) + roi_top_offset

                tl_o, tr_o, br_o, bl_o = (to_orig(tl), to_orig(tr),
                                           to_orig(br), to_orig(bl))
                top_y = float(min(tl_o[1], tr_o[1]))
                bot_y = float(max(br_o[1], bl_o[1]))
                cx    = int(np.mean(pts[:, 0]))

                if br[1] >= h_roi - 2 or bl[1] >= h_roi - 2:
                    bot_y = float(roi_top_offset + h_roi)

                with self._lock:
                    self._marker.found = True
                    self._marker.id    = int(mid)
                    self._marker.top_y = top_y
                    self._marker.bot_y = bot_y
                    self._marker.cx    = cx
                    self._marker.tl    = tl_o
                    self._marker.tr    = tr_o
                found = True
                break

            if not found:
                self.get_logger().debug(
                    f"ArUco 감지됨 but 엔드포인트 불일치: "
                    f"{ids.flatten().tolist()} / endpoints={self.endpoint_ids}")

        if not found:
            with self._lock:
                self._marker.found = False

    def _detect_line(self, roi: np.ndarray, frame_w: int) -> None:
        """
        색상 라인 감지.

        [B2] BLUE 복귀 시 3중 방어 적용:
          방어 1: COLOR_HSV['BLUE_RETURN'] S≥50, V≥60 (채도 저하 대응)
          방어 2: MORPH_OPEN 5×5 커널 (소면적 잡음 억제)
          방어 3: m00 임계값 300 (도트·반사 잡음의 m00<300 으로 제거)

        RED·YELLOW 및 BLUE 접근 시: v7.0 동작과 완전 동일.
        """
        if self.target_color is None:
            with self._lock:
                self.line_detected = False
                self.line_cx       = None
            return

        with self._lock:
            returning = self.returning

        # [B2] BLUE 복귀 여부에 따라 HSV 키·커널·임계값 분기
        if self.target_color == 'BLUE' and returning:
            hsv_key    = 'BLUE_RETURN'
            kernel_sz  = self.LINE_MORPH_RETURN      # 5
            m00_min    = self.LINE_M00_RETURN         # 300
        else:
            hsv_key    = self.target_color
            kernel_sz  = self.LINE_MORPH_APPROACH    # 3
            m00_min    = self.LINE_M00_APPROACH       # 125

        small = cv2.resize(roi, None, fx=0.5, fy=0.5,
                           interpolation=cv2.INTER_LINEAR)
        hsv   = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)

        masks = [cv2.inRange(hsv, np.array(lo), np.array(hi))
                 for lo, hi in COLOR_HSV[hsv_key]]
        mask  = masks[int(np.argmax([cv2.countNonZero(m) for m in masks]))]
        mask  = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                 np.ones((kernel_sz, kernel_sz), np.uint8))

        M = cv2.moments(mask)
        if M['m00'] < m00_min:
            with self._lock:
                self.line_detected = False
                self.line_cx       = None
        else:
            with self._lock:
                self.line_detected = True
                self.line_cx       = int(M['m10'] / M['m00']) * 2

    # ════════════════════════════════════════════════════════════════════════
    #  헬퍼 — 센서
    # ════════════════════════════════════════════════════════════════════════

    @staticmethod
    def _lidar_valid_ranges(msg: LaserScan, angle_deg: float) -> List[float]:
        n     = len(msg.ranges)
        step  = msg.angle_increment
        ang   = np.deg2rad(angle_deg)
        idx_r = int(ang / step) % n
        idx_l = int((2 * np.pi - ang) / step) % n
        raw   = list(msg.ranges[:idx_r + 1]) + list(msg.ranges[idx_l:])
        return [r for r in raw
                if math.isfinite(r) and msg.range_min < r < msg.range_max]

    def _get_lidar_narrow_median(self, n_samples: Optional[int] = None) -> float:
        n_samples = n_samples or self.LIDAR_MEDIAN_N
        samples: List[float] = []
        for _ in range(n_samples):
            with self._lock:
                d = self._lidar.narrow_dist
            if d != float('inf'):
                samples.append(d)
            time.sleep(0.13)

        if not samples:
            self.get_logger().warn("⚠️  _get_lidar_narrow_median: 유효 샘플 없음")
            return float('inf')

        median = float(np.median(samples))
        self.get_logger().info(
            f"📡 LiDAR ±5° 중앙값: {median:.4f} m  ({len(samples)} 샘플)")
        return median

    def _get_odom_pos(self) -> Tuple[float, float]:
        with self._lock:
            return self._odom.x, self._odom.y

    def _odom_dist_from(self, x0: float, y0: float) -> float:
        x, y = self._get_odom_pos()
        return math.sqrt((x - x0) ** 2 + (y - y0) ** 2)

    def _get_yaw(self) -> float:
        with self._lock:
            oz, ow = self._odom.oz, self._odom.ow
        return 2.0 * math.atan2(oz, ow)

    @staticmethod
    def _yaw_diff(yaw_start: float, yaw_end: float) -> float:
        diff = yaw_end - yaw_start
        return math.atan2(math.sin(diff), math.cos(diff))

    def _wait_odom_stable(self) -> bool:
        self.get_logger().info(
            f"⏳ /odom yaw 안정화 대기 중 "
            f"(허용 변화량 ±{self.ODOM_STABLE_TOL_DEG:.1f}°/0.5s, "
            f"최대 {self.ODOM_STABLE_TIMEOUT:.1f} s)")

        end = time.time() + self.ODOM_STABLE_TIMEOUT
        while time.time() < end and rclpy.ok():
            y0 = self._get_yaw()
            time.sleep(0.5)
            delta_deg = abs(math.degrees(self._yaw_diff(y0, self._get_yaw())))
            if delta_deg < self.ODOM_STABLE_TOL_DEG:
                self.get_logger().info(
                    f"✅ /odom yaw 안정화 완료  변화량={delta_deg:.2f}°")
                return True
            self.get_logger().debug(f"   yaw 변화량={delta_deg:.2f}° — 대기 중")

        self.get_logger().warn(
            f"⚠️  /odom yaw 안정화 타임아웃 ({self.ODOM_STABLE_TIMEOUT:.1f} s) "
            f"— 미안정 상태로 진행")
        return False

    # ── 회전 슬립 계측 ───────────────────────────────────────────────────────

    def _try_commit_turn_slip(self, yaw_start: float,
                              yaw_now: float, cmd_deg: float) -> bool:
        actual_rad = abs(self._yaw_diff(yaw_start, yaw_now))
        actual_deg = math.degrees(actual_rad)

        if actual_deg < self.TURN_MIN_DEG:
            self.get_logger().warn(
                f"⚠️  회전 슬립 계측 기각: 실제 {actual_deg:.1f}° < {self.TURN_MIN_DEG}°")
            return False

        self._slip.turn_correction = math.radians(cmd_deg) / actual_rad
        self._slip.turn_measured   = True

        error_deg = cmd_deg - actual_deg
        if abs(error_deg) > self.TURN_SLIP_TOL_DEG:
            self.get_logger().warn(
                f"⚠️  슬립 계측 오차 큼: 명령 {cmd_deg:.1f}°  실제 {actual_deg:.1f}°  "
                f"오차={error_deg:+.1f}° (허용 ±{self.TURN_SLIP_TOL_DEG:.1f}°)  "
                f"보정 계수={self._slip.turn_correction:.4f}")
        else:
            self.get_logger().info(
                f"📐 회전 슬립 계측 완료  명령 {cmd_deg:.1f}°  실제 {actual_deg:.1f}°  "
                f"오차={error_deg:+.1f}°  보정 계수={self._slip.turn_correction:.4f}")
        return True

    def _measure_turn_slip_supplement(self, yaw_before: float) -> None:
        """조기 라인 발견(90° 미만) 시 CW/CCW 각 45° 추가 계측 후 평균."""
        supp_deg  = self.TURN_SUPPLEMENT_DEG
        supp_rad  = math.radians(supp_deg)
        supp_time = supp_rad / self.SPD_SEARCH

        self.get_logger().info(
            f"📐 조기 발견 — CW +{supp_deg:.0f}° / CCW +{supp_deg:.0f}° 각각 계측 후 평균")

        self._ignore_camera_event.set()

        yaw_cw_start = self._get_yaw()
        self._timed_move(0.0, -self.SPD_SEARCH, supp_time)
        actual_cw_rad  = abs(self._yaw_diff(yaw_cw_start, self._get_yaw()))

        yaw_ccw_start = self._get_yaw()
        self._timed_move(0.0,  self.SPD_SEARCH, supp_time)
        actual_ccw_rad = abs(self._yaw_diff(yaw_ccw_start, self._get_yaw()))

        self._ignore_camera_event.clear()

        actual_cw_deg  = math.degrees(actual_cw_rad)
        actual_ccw_deg = math.degrees(actual_ccw_rad)

        self.get_logger().info(
            f"   CW  계측: 명령 {supp_deg:.1f}°  실제 {actual_cw_deg:.1f}°\n"
            f"   CCW 계측: 명령 {supp_deg:.1f}°  실제 {actual_ccw_deg:.1f}°")

        if (actual_cw_deg  < self.TURN_SUPPLEMENT_MIN_DEG or
                actual_ccw_deg < self.TURN_SUPPLEMENT_MIN_DEG):
            self.get_logger().warn(
                f"⚠️  추가 회전 계측 기각: CW={actual_cw_deg:.1f}° / "
                f"CCW={actual_ccw_deg:.1f}° — "
                f"{self.TURN_SUPPLEMENT_MIN_DEG:.1f}° 미만 존재")
            return

        correction_cw  = supp_rad / actual_cw_rad
        correction_ccw = supp_rad / actual_ccw_rad
        self._slip.turn_correction = (correction_cw + correction_ccw) / 2.0
        self._slip.turn_measured   = True
        self.get_logger().info(
            f"📐 추가 회전 슬립 계측 완료  "
            f"CW={correction_cw:.4f}  CCW={correction_ccw:.4f}  "
            f"평균={self._slip.turn_correction:.4f}")

    # ════════════════════════════════════════════════════════════════════════
    #  헬퍼 — 구동
    # ════════════════════════════════════════════════════════════════════════

    @staticmethod
    def _angular(cx: int, frame_w: int, gain: float) -> float:
        """화면 중심 기준 편차 → 각속도 변환 (라인·마커 공용)."""
        return -float(cx - frame_w // 2) / (frame_w / 2) * gain

    def _stop(self) -> None:
        self.pub_vel.publish(Twist())

    def _move(self, lin: float, ang: float) -> None:
        t = Twist()
        t.linear.x  = float(lin)
        t.angular.z = float(ang)
        self.pub_vel.publish(t)

    def _timed_move(self, lin: float, ang: float, duration: float) -> None:
        end = time.time() + duration
        while time.time() < end and rclpy.ok():
            self._move(lin, ang)
            time.sleep(0.05)
        self._stop()

    def _sleep_ok(self, sec: float) -> None:
        end = time.time() + sec
        while time.time() < end and rclpy.ok():
            time.sleep(0.05)

    def _change_state(self, new: str) -> None:
        if self.state != new:
            self.get_logger().info(f"🔄  {self.state} → {new}")
        self.state = new

    def _publish_string(self, publisher, data: str) -> None:
        msg      = String()
        msg.data = data
        publisher.publish(msg)

    def _wait_event_frames(self, n: int = 2, timeout: float = 1.0) -> None:
        for _ in range(n):
            self._frame_event.clear()
            self._frame_event.wait(timeout=timeout)

    # ════════════════════════════════════════════════════════════════════════
    #  FSM 루프
    # ════════════════════════════════════════════════════════════════════════

    def _fsm_loop(self) -> None:
        _DISPATCH = {
            St.WAIT:           self._s_wait,
            St.IDLE:           self._s_idle,
            St.SEARCH:         self._s_search,
            St.APPROACH:       self._s_approach,
            St.APPROACH_ALIGN: self._s_approach_align,
            St.EMERGENCY_STOP: self._s_estop,
            St.UNLOAD_ALIGN:   self._s_unload_align,
            St.UNLOAD_STANDBY: self._s_unload_standby,
            St.RETURN:         self._s_return,
            St.DONE:           self._s_done,
        }
        while rclpy.ok():
            _DISPATCH.get(self.state, self._s_wait)()
            time.sleep(0.05)

    # ════════════════════════════════════════════════════════════════════════
    #  FSM 상태 핸들러 (v7.0 과 동일)
    # ════════════════════════════════════════════════════════════════════════

    def _s_wait(self) -> None:
        time.sleep(0.1)

    def _s_idle(self) -> None:
        self._stop()
        time.sleep(0.1)

    def _s_search(self) -> None:
        """제자리 CW → CCW 각 1바퀴 라인 탐색. CW 구간 회전 슬립 1회 계측."""
        self.get_logger().info(f"🔍 SEARCH — 목표 색상: {self.target_color}")

        if not self._slip.turn_measured:
            self._wait_odom_stable()

        SEARCH_TURN_FACTOR = 4.0
        one_turn = (SEARCH_TURN_FACTOR * np.pi) / self.SPD_SEARCH

        for direction, label in ((1.0, 'CW'), (-1.0, 'CCW')):
            yaw_start  = self._get_yaw()
            end        = time.time() + one_turn
            do_measure = (label == 'CW' and not self._slip.turn_measured)

            while time.time() < end and rclpy.ok():
                if self.state != St.SEARCH:
                    return
                if self.line_detected:
                    self._stop()
                    if do_measure:
                        yaw_now    = self._get_yaw()
                        actual_deg = abs(math.degrees(
                            self._yaw_diff(yaw_start, yaw_now)))
                        if actual_deg >= self.TURN_MIN_DEG:
                            self._try_commit_turn_slip(yaw_start, yaw_now, actual_deg)
                        else:
                            self._measure_turn_slip_supplement(yaw_now)
                    self.get_logger().info(f"✅ SEARCH({label}): 라인 발견 → APPROACH")
                    self._change_state(St.APPROACH)
                    return
                self._move(0.0, -self.SPD_SEARCH * direction)
                time.sleep(0.05)

            self._stop()
            if do_measure and not self._slip.turn_measured:
                yaw_now    = self._get_yaw()
                actual_deg = abs(math.degrees(self._yaw_diff(yaw_start, yaw_now)))
                self._try_commit_turn_slip(yaw_start, yaw_now, actual_deg)
            self._sleep_ok(0.3)

        self.get_logger().warn("⚠️  SEARCH: 라인 미발견 — 1 s 후 재탐색")
        self._sleep_ok(1.0)

    def _s_approach(self) -> None:
        """
        v7.0 동작 그대로 유지.
        returning=True  → 라인트레이싱 우선 (귀환 모드)
        returning=False → 마커 인식 최우선, 미인식 시 라인트레이싱
        """
        with self._lock:
            returning  = self.returning
            found      = self._marker.found
            bot_y      = self._marker.bot_y
            marker_cx  = self._marker.cx
            line_det   = self.line_detected
            line_cx    = self.line_cx
            frame_w    = self.FRAME_W

        # ── 귀환 모드 ────────────────────────────────────────────────────────
        if returning:
            if not line_det:
                self._stop()
                self.get_logger().info("🏁 귀환 중 라인 소실 → DONE")
                self._change_state(St.DONE)
                return
            if abs(line_cx - frame_w // 2) > frame_w * 0.10:
                self._stop()
                self._change_state(St.APPROACH_ALIGN)
                return
            self._move(self.SPD_LINE, self._angular(line_cx, frame_w, self.SPD_ANG))
            return

        # ── 마커 우선 모드 ───────────────────────────────────────────────────
        if found:
            self._stop()
            if bot_y >= self.MARKER_TOO_CLOSE_Y:
                self.get_logger().warn(
                    f"↩️  너무 가까움 bot_y={bot_y:.0f} — 후진 1.0 s")
                self._timed_move(-self.SPD_BACK, 0.0, 1.0)
                return
            if bot_y >= self.MARKER_STOP_BOT_Y:
                self.get_logger().info(
                    f"🎯 정렬 범위 도달 bot_y={bot_y:.0f} → UNLOAD_ALIGN")
                self._change_state(St.UNLOAD_ALIGN)
                return
            self._move(self.SPD_LINE,
                       self._angular(marker_cx, frame_w, self.SPD_ANG))
            return

        # ── 라인트레이싱 폴백 ────────────────────────────────────────────────
        if not line_det:
            self._stop()
            self.get_logger().warn("⚠️  APPROACH: 라인 소실 → SEARCH")
            self._change_state(St.SEARCH)
            return
        if abs(line_cx - frame_w // 2) > frame_w * 0.10:
            self._stop()
            self._change_state(St.APPROACH_ALIGN)
            return
        self._move(self.SPD_LINE, self._angular(line_cx, frame_w, self.SPD_ANG))

    def _s_approach_align(self) -> None:
        """라인 중앙 정렬. returning=True 시 마커 체크 생략."""
        with self._lock:
            returning = self.returning
            found     = self._marker.found
            line_det  = self.line_detected
            line_cx   = self.line_cx
            frame_w   = self.FRAME_W

        if not returning and found:
            self._stop()
            self._change_state(St.APPROACH)
            return
        if not line_det:
            self._stop()
            self._change_state(St.DONE if returning else St.SEARCH)
            return

        err = line_cx - frame_w // 2
        if abs(err) <= frame_w * 0.05:
            self._stop()
            self.get_logger().info("✅ APPROACH_ALIGN: 정렬 완료 → APPROACH")
            self._change_state(St.APPROACH)
            return
        self._move(self.SPD_ALIGN, self._angular(line_cx, frame_w, self.SPD_ANG))

    def _s_estop(self) -> None:
        self._stop()
        with self._lock:
            obstacle = self._lidar.obstacle_front
            prev     = self._lidar.pre_estop_state
        if not obstacle:
            self.get_logger().info(f"✅ EMERGENCY_STOP 해제 → {prev}")
            self._change_state(prev)

    def _s_unload_align(self) -> None:
        """ArUco 기준 정밀 정렬."""
        self.get_logger().info("🔧 UNLOAD_ALIGN: 정밀 정렬 시작")
        MAX_ITER      = 40
        FRAME_TIMEOUT = 1.0

        for it in range(MAX_ITER):
            if not rclpy.ok():
                return

            self._ignore_camera_event.set()
            self._frame_event.clear()
            self._sleep_ok(0.05)
            self._ignore_camera_event.clear()

            if not self._frame_event.wait(timeout=FRAME_TIMEOUT):
                self.get_logger().warn(f"⚠️  UNLOAD_ALIGN iter={it}: 프레임 타임아웃")
                continue

            with self._lock:
                found = self._marker.found
                tl    = self._marker.tl
                tr    = self._marker.tr
                cx    = self._marker.cx

            if not found:
                self.get_logger().warn(
                    f"⚠️  UNLOAD_ALIGN iter={it}: 마커 미인식 → 후진 0.2 s")
                self._timed_move(-self.SPD_LINE, 0.0, 0.2)
                continue

            _, tl_y   = tl
            _, tr_y   = tr
            top_delta = abs(tl_y - tr_y)
            top_ok    = top_delta <= self.ALIGN_TOP_TOL
            cx_ok     = self.ALIGN_CX_MIN <= cx <= self.ALIGN_CX_MAX

            self.get_logger().info(
                f"  [iter={it}] top_delta={top_delta:.1f}px(ok={top_ok})  "
                f"cx={cx}(ok={cx_ok})")

            if top_ok and cx_ok:
                self._stop()
                self.get_logger().info("✅ UNLOAD_ALIGN: 조건 충족 → UNLOAD_STANDBY")
                self._change_state(St.UNLOAD_STANDBY)
                return

            if not cx_ok:
                err = cx - self.FRAME_W // 2
                ang = -float(np.sign(err)) * 0.15
                self.get_logger().info(
                    f"  → cx 보정: err={err}px  ang={ang:.2f}  0.25 s")
                self._timed_move(0.0, ang, 0.25)

            if not top_ok:
                tilt = tl_y - tr_y
                ang  = float(np.sign(tilt)) * 0.10
                self.get_logger().info(
                    f"  → 수평 보정: tilt={tilt:.1f}px  ang={ang:.2f}  0.20 s")
                self._timed_move(0.0, ang, 0.20)

        self.get_logger().warn("⚠️  UNLOAD_ALIGN: 최대 반복 → 강제 UNLOAD_STANDBY")
        self._change_state(St.UNLOAD_STANDBY)

    # ────────────────────────────────────────────────────────────────────────
    def _s_unload_standby(self) -> None:
        if not self._slip.fwd_measured:
            self._unload_phase1_lidar()
        else:
            self._unload_phase2_cross()

    def _unload_phase1_lidar(self) -> None:
        """UNLOAD_ALIGN 완료 직후 B 측정 → LiDAR 28 cm 도달까지 전진."""
        B = self._get_lidar_narrow_median()

        if B == float('inf') or B <= self.UNLOAD_TARGET_DIST + 0.05:
            self.get_logger().warn(
                f"⚠️  1단계: B={B:.3f} m — 유효 범위 밖 → fallback 7.0 s 전진")
            self._timed_move(self.SPD_LINE, 0.0, 7.0)
            self._do_fork_and_return()
            return

        E = B - self.UNLOAD_TARGET_DIST
        self.get_logger().info(
            f"📦 UNLOAD_STANDBY [1단계]\n"
            f"   B={B:.4f} m  E={E:.4f} m  "
            f"목표 → {self.UNLOAD_TARGET_DIST*100:.0f} cm")

        x0, y0  = self._get_odom_pos()
        t_start = time.time()
        timeout = (E / self.SPD_LINE) * 2.5
        reached = False

        while rclpy.ok():
            elapsed = time.time() - t_start
            with self._lock:
                d_now = self._lidar.narrow_dist
            if d_now <= self.UNLOAD_TARGET_DIST:
                reached = True
                break
            if elapsed >= timeout:
                self.get_logger().warn(
                    f"⚠️  1단계 타임아웃 ({timeout:.1f} s)  LiDAR={d_now:.3f} m")
                break
            self._move(self.SPD_LINE, 0.0)
            time.sleep(0.05)

        self._stop()

        fwd_t = time.time() - t_start
        fwd_d = self._odom_dist_from(x0, y0)

        self._slip.fwd_t        = fwd_t
        self._slip.fwd_d        = fwd_d
        self._slip.fwd_ratio    = (1.0 - fwd_d / E) if E > 0.001 else 0.0
        self._slip.fwd_measured = True

        self.get_logger().info(
            f"✅ 1단계 완료\n"
            f"   fwd_t={fwd_t:.3f} s\n"
            f"   fwd_d={fwd_d:.4f} m\n"
            f"   E={E:.4f} m  slip={self._slip.fwd_ratio:.4f}  "
            f"도달: {'성공' if reached else '타임아웃'}")

        self._do_fork_and_return()

    def _unload_phase2_cross(self) -> None:
        """
        fwd_t + fwd_d 교차 감시 전진.

        정지 우선순위:
          P1. LiDAR < 27 cm + odom > 5 mm → 비상 정지
          P2. elapsed ≥ fwd_t AND odom ≥ fwd_d → 정상 완료
          P3. elapsed ≥ fwd_t, odom 부족 ≤ 3 cm → 추가 전진
          P4. elapsed ≥ fwd_t, odom 부족 > 3 cm → 강제 정지
          P5. odom ≥ fwd_d, elapsed < fwd_t → 조기 정지
        """
        time_target = self._slip.fwd_t
        odom_target = self._slip.fwd_d

        self.get_logger().info(
            f"📦 UNLOAD_STANDBY [2단계]\n"
            f"   time_target={time_target:.3f} s  "
            f"odom_target={odom_target:.4f} m  "
            f"slip={self._slip.fwd_ratio:.4f} (진단용)")

        x0, y0      = self._get_odom_pos()
        t_start     = time.time()
        stop_reason = 'none'

        while rclpy.ok():
            elapsed    = time.time() - t_start
            odom_moved = self._odom_dist_from(x0, y0)
            with self._lock:
                front_dist = self._lidar.narrow_dist

            # P1: 비상 정지
            if (front_dist < self.LIDAR_UNLOAD_STOP
                    and odom_moved > self.LIDAR_UNLOAD_MIN_ODOM):
                stop_reason = 'lidar_estop'
                self.get_logger().warn(
                    f"🚨 UNLOAD E-STOP: LiDAR={front_dist:.3f} m  "
                    f"포크 끝단 → 벽 ≈ {(front_dist - 0.22)*100:.1f} cm  "
                    f"odom={odom_moved:.4f} m")
                break

            # P2: 정상 완료
            if elapsed >= time_target and odom_moved >= odom_target:
                stop_reason = 'normal'
                self.get_logger().info(
                    f"✅ 정상 완료: elapsed={elapsed:.3f} s  odom={odom_moved:.4f} m")
                break

            # P3/P4: 시간 초과 후 odom 잔여량 분기
            if elapsed >= time_target:
                gap = odom_target - odom_moved
                if gap > 0.03:
                    stop_reason = 'odom_gap_exceed'
                    self.get_logger().warn(
                        f"⚠️  odom 부족 gap={gap*100:.1f} cm > 3 cm → 강제 정지")
                    break
                else:
                    self._move(self.SPD_LINE, 0.0)
                    time.sleep(0.05)
                    continue

            # P5: 조기 정지
            if odom_moved >= odom_target:
                stop_reason = 'odom_early'
                self.get_logger().info(
                    f"✅ 조기 odom 달성: {odom_moved:.4f} m / {elapsed:.3f} s")
                break

            self._move(self.SPD_LINE, 0.0)
            time.sleep(0.05)

        self._stop()
        self.get_logger().info(f"🏁 2단계 완료 — 정지 사유: {stop_reason}")
        self._do_fork_and_return()

    # ────────────────────────────────────────────────────────────────────────
    def _do_fork_and_return(self) -> None:
        """포크 DOWN → /fork_done 대기 → 후진 5 s → RETURN."""
        FORK_TIMEOUT = 10.0
        with self._lock:
            self.fork_done = False

        self._publish_string(self.pub_fork, 'DOWN')
        self.get_logger().info("🔽 /fork_cmd: DOWN 발행")

        wait_start = time.time()
        while rclpy.ok():
            with self._lock:
                done = self.fork_done
            if done:
                self.get_logger().info("✅ /fork_done 수신")
                break
            if time.time() - wait_start > FORK_TIMEOUT:
                self.get_logger().warn(
                    f"⚠️  /fork_done 타임아웃 ({FORK_TIMEOUT:.0f} s) — 강제 진행")
                break
            time.sleep(0.05)

        self.get_logger().info("↩️  후진 5.0 s")
        self._timed_move(-self.SPD_LINE, 0.0, 5.0)
        self._change_state(St.RETURN)

    def _s_return(self) -> None:
        """180° 회전 후 귀환 (v7.0 동일)."""
        with self._lock:
            self.endpoint_ids -= {0, 1, 2}
            self.returning = True
        self.get_logger().info(
            f"🗑️  엔드포인트 0·1·2 제거 — 잔여: {self.endpoint_ids}  [RETURNING ON]")

        TARGET_RAD  = np.pi
        CORRECT_TOL = math.radians(self.RETURN_CORRECT_TOL_DEG)
        TRIM_TARGET = TARGET_RAD * 0.97

        correction = (self._slip.turn_correction
                      if self._slip.turn_measured
                      else self._slip.turn_fallback)
        half_turn  = (TARGET_RAD / self.SPD_ANG) * correction

        self.get_logger().info(
            f"↩️  RETURN: 180° 회전  보정 계수={correction:.4f}  "
            f"명령 시간={half_turn:.2f} s  "
            f"({'실측' if self._slip.turn_measured else 'fallback'})")

        yaw_start = self._get_yaw()
        self._ignore_camera_event.set()
        self._timed_move(0.0, self.SPD_ANG, half_turn)

        yaw_after      = self._get_yaw()
        signed_rotated = self._yaw_diff(yaw_start, yaw_after)
        rotated        = abs(signed_rotated)
        turn_dir       = 1.0 if signed_rotated >= 0 else -1.0
        error_rad      = TARGET_RAD - rotated
        error_deg      = math.degrees(error_rad)

        self.get_logger().info(
            f"📐 교차 검증: 목표={math.degrees(TARGET_RAD):.1f}°  "
            f"실측={math.degrees(rotated):.1f}°  오차={error_deg:+.1f}°")

        if abs(error_rad) <= CORRECT_TOL:
            self.get_logger().info(
                f"✅ 교차 검증 통과 — 보정 불필요 "
                f"(허용 ±{self.RETURN_CORRECT_TOL_DEG:.1f}°)")
        else:
            self._trim_rotation(yaw_start, turn_dir, error_rad,
                                error_deg, TRIM_TARGET)

        self._ignore_camera_event.clear()

        self.get_logger().info("⏳ 카메라 프레임 안정화 대기 (2프레임)...")
        self._wait_event_frames(n=2, timeout=1.0)

        self.get_logger().info("✅ RETURN: 회전 완료 → 귀환")
        self._change_state(St.APPROACH)

    def _trim_rotation(self, yaw_start: float, turn_dir: float,
                       error_rad: float, error_deg: float,
                       trim_target: float) -> None:
        """yaw 보정 루프 (부족 → 추가 회전, 과회전 → 역방향)."""
        trim_dir   = turn_dir if error_rad > 0 else -turn_dir
        t_trim_end = time.time() + self.RETURN_TRIM_TIMEOUT

        self.get_logger().info(
            f"🔧 보정 회전: {'부족 → 추가' if error_rad > 0 else '과회전 → 역방향'}  "
            f"오차={error_deg:+.1f}°  최대 대기={self.RETURN_TRIM_TIMEOUT:.1f} s")

        while time.time() < t_trim_end and rclpy.ok():
            current_rotated = abs(self._yaw_diff(yaw_start, self._get_yaw()))
            if current_rotated >= trim_target:
                self.get_logger().info(
                    f"✅ 보정 중 목표 yaw 도달  "
                    f"실측={math.degrees(current_rotated):.1f}°")
                break
            self._move(0.0, self.SPD_ANG * trim_dir)
            time.sleep(0.05)
        else:
            self.get_logger().warn(
                f"⚠️  보정 TRIM_TIMEOUT ({self.RETURN_TRIM_TIMEOUT:.1f} s) 초과 "
                f"— 강제 정지")

        self._stop()
        rotated_fin = abs(self._yaw_diff(yaw_start, self._get_yaw()))
        self.get_logger().info(
            f"✅ 보정 완료: 최종 실측={math.degrees(rotated_fin):.1f}°")

    def _s_done(self) -> None:
        self.get_logger().info("🏁 DONE — 미션 완료")
        with self._lock:
            self.target_color  = None
            self.endpoint_ids  = {100}
            self.returning     = False
            self._marker.found = False
        self._stop()
        self._publish_string(self.pub_done, 'Done')
        self._change_state(St.IDLE)
        time.sleep(0.1)
        self._stop()
        self._change_state(St.WAIT)


# ════════════════════════════════════════════════════════════════════════════
#  ArUco 검출기 싱글턴
# ════════════════════════════════════════════════════════════════════════════
def _build_aruco_detector() -> cv2.aruco.ArucoDetector:
    _dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    _params = cv2.aruco.DetectorParameters()
    _params.adaptiveThreshWinSizeMax    = 7
    _params.adaptiveThreshWinSizeStep   = 4
    _params.minMarkerPerimeterRate      = 0.02
    _params.polygonalApproxAccuracyRate = 0.05
    return cv2.aruco.ArucoDetector(_dict, _params)


_ARUCO_DETECTOR = _build_aruco_detector()


# ════════════════════════════════════════════════════════════════════════════
def main(args=None) -> None:
    rclpy.init(args=args)
    node = FSMUnloadNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()