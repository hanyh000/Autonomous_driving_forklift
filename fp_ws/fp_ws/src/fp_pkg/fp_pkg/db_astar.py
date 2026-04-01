import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String, Bool, Int32
from math import atan2, sqrt, sin, pi
import heapq
import numpy as np
from sensor_msgs.msg import LaserScan, BatteryState
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import threading
import sys
import tty
import termios

sys.path.append('/home/dev/fp_ws/src/fp_pkg')
from database.node_service import NodeService
from database.map_service import map_service

import threading
import time
import copy
from dataclasses import dataclass, field
from typing import Optional, List, Tuple
from math import sqrt


class RobotState:
    IDLE            = 'idle'
    MISSION         = 'mission'
    WAYPOINT        = 'waypoint'
    LOADING         = 'loading'
    UNLOADING       = 'unloading'
    ABORTING        = 'aborting'
    RUNNING_TO_HOME = 'running_to_home'
    RETURNING       = 'returning'
    RESUMING        = 'resuming'
    CHARGING        = 'charging'
    EMERGENCY       = 'emergency'
    SAFE_EXIT       = 'safe_exit'


VALID_TRANSITIONS = {
    RobotState.IDLE:            [RobotState.MISSION, RobotState.WAYPOINT,
                                  RobotState.CHARGING, RobotState.EMERGENCY,
                                  RobotState.ABORTING, RobotState.RESUMING,
                                  RobotState.UNLOADING],
    RobotState.MISSION:         [RobotState.LOADING, RobotState.UNLOADING,
                                  RobotState.ABORTING, RobotState.CHARGING,
                                  RobotState.EMERGENCY, RobotState.IDLE],
    RobotState.WAYPOINT:        [RobotState.IDLE, RobotState.ABORTING,
                                  RobotState.CHARGING, RobotState.EMERGENCY],
    RobotState.LOADING:         [RobotState.MISSION, RobotState.UNLOADING,
                                  RobotState.ABORTING, RobotState.EMERGENCY],
    RobotState.UNLOADING:       [RobotState.MISSION, RobotState.ABORTING,
                                  RobotState.EMERGENCY],
    RobotState.ABORTING:        [RobotState.RUNNING_TO_HOME,
                                  RobotState.SAFE_EXIT, RobotState.EMERGENCY],
    RobotState.RUNNING_TO_HOME: [RobotState.IDLE, RobotState.RESUMING,
                                  RobotState.EMERGENCY, RobotState.SAFE_EXIT],
    RobotState.RETURNING:       [RobotState.IDLE, RobotState.RESUMING,
                                  RobotState.EMERGENCY, RobotState.SAFE_EXIT],
    RobotState.RESUMING:        [RobotState.RUNNING_TO_HOME, RobotState.RETURNING,
                                  RobotState.EMERGENCY, RobotState.SAFE_EXIT],
    RobotState.CHARGING:        [RobotState.RETURNING, RobotState.IDLE,
                                  RobotState.EMERGENCY],
    RobotState.EMERGENCY:       [RobotState.RESUMING, RobotState.IDLE,
                                  RobotState.SAFE_EXIT],
    RobotState.SAFE_EXIT:       [RobotState.IDLE],
}


class ArduinoCmd:
    LOAD = 'LOAD'
    UNLD = 'UNLD'
    HOME = 'HOME'
    STOP = 'STOP'


@dataclass
class MissionSnapshot:
    robot_state: str = RobotState.IDLE
    current_node_key: str = 'standby'
    pending_abort: bool = False
    mission_idx: int = 0
    mission_sequence: List = field(default_factory=list)
    current_zone: int = 1
    current_load_key: str = 'load_1'
    current_unload_key: str = 'unload_1'
    current_room_type: Optional[str] = None
    is_waypoint_loop: bool = False
    wp_loop_idx: int = 0
    wp_final_pose: Optional[List] = None
    wp_target_yaw: float = 0.0
    pose_x: float = 0.0
    pose_y: float = 0.0
    pose_yaw: float = 0.0
    global_path: List = field(default_factory=list)
    path_index: int = 0
    timestamp: float = 0.0
    abort_reason: str = ''
    abort_pose: Optional[List] = None


class ReturnManager:
    SAFE_FRONT_DIST  = 0.40
    SAFE_SIDE_DIST   = 0.25
    SCAN_MAX_AGE     = 0.5
    PRECHECK_TIMEOUT = 10.0
    MAX_RETRY        = 3
    SAFE_EXIT_INTERVAL = 1.0

    def __init__(self, node):
        self._node = node
        self._state_lock = threading.Lock()
        self.robot_state = RobotState.IDLE
        self.current_node_key = 'standby'
        self._snapshot: Optional[MissionSnapshot] = None
        self._return_sequence: List[Tuple[str, list]] = []
        self._return_idx: int = 0
        self._pending_abort: bool = False
        self._retry_count: int = 0
        self._precheck_start: Optional[float] = None
        self._precheck_resume_state: str = RobotState.RUNNING_TO_HOME
        self.RETURN_NEXT: dict = {}
        self._path_calc_thread: Optional[threading.Thread] = None
        self._safe_exit_thread: Optional[threading.Thread] = None
        self._safe_exit_active: bool = False

    def transition(self, new_state: str) -> bool:
        with self._state_lock:
            allowed = VALID_TRANSITIONS.get(self.robot_state, [])
            if new_state not in allowed:
                self._node.get_logger().error(
                    f"❌ 전이 차단: {self.robot_state} → {new_state}")
                return False
            prev = self.robot_state
            self.robot_state = new_state
        self._node.get_logger().info(f"[FSM] State Changed: {prev} → {new_state}")
        if new_state == RobotState.SAFE_EXIT:
            self._node.stop_robot()
            # SAFE_EXIT 진입 시 1회 stop 플래그는 이미 stop_robot()으로 처리됨
            # control_loop에서 중복 발행 방지를 위해 플래그 초기화
            self._node._safe_exit_stop_sent = True
            self._start_safe_exit_loop()
        return True

    def get_state(self) -> str:
        with self._state_lock:
            return self.robot_state

    def update_node_key(self, key: str):
        with self._state_lock:
            self.current_node_key = key

    def build_return_graph(self, waypoints: list):
        n = len(waypoints)
        self.RETURN_NEXT = {}
        for i in range(n):
            key = f'wp_{i + 1}'
            self.RETURN_NEXT[key] = f'wp_{i}' if i > 0 else 'standby'
        self.RETURN_NEXT['wp_final'] = f'wp_{n}' if n > 0 else 'standby'
        self.RETURN_NEXT['unload']  = f'wp_{n}' if n > 0 else 'standby'
        self.RETURN_NEXT['load']    = 'standby'
        self.RETURN_NEXT['charge']  = 'standby'
        self.RETURN_NEXT['standby'] = None
        self._node.get_logger().info(f"🗺️  RETURN_NEXT: {self.RETURN_NEXT}")

    def take_snapshot(self, abort_reason: str = ''):
        node = self._node
        try:
            pose_x = node.current_pose[0] if node.current_pose else 0.0
            pose_y = node.current_pose[1] if node.current_pose else 0.0
            buf = MissionSnapshot(
                robot_state=self.robot_state,
                current_node_key=self.current_node_key,
                pending_abort=self._pending_abort,
                mission_idx=node.mission_idx,
                mission_sequence=copy.deepcopy(node.mission_sequence),
                current_zone=node.current_zone,
                current_load_key=node.current_load_key,
                current_unload_key=node.current_unload_key,
                current_room_type=node.current_room_type,
                is_waypoint_loop=node.is_waypoint_loop,
                wp_loop_idx=node.wp_loop_idx,
                wp_final_pose=copy.deepcopy(node.wp_final_pose),
                wp_target_yaw=node.wp_target_yaw,
                pose_x=pose_x,
                pose_y=pose_y,
                pose_yaw=node.current_yaw,
                global_path=copy.deepcopy(node.global_path),
                path_index=node.path_index,
                timestamp=time.time(),
                abort_reason=abort_reason,
                abort_pose=[pose_x, pose_y],
            )
            with self._state_lock:
                self._snapshot = buf
            node.get_logger().info(
                f"📸 스냅샷 | 노드={buf.current_node_key} "
                f"위치=({pose_x:.2f},{pose_y:.2f}) 원인={abort_reason}")
        except Exception as e:
            node.get_logger().error(f"❌ 스냅샷 실패: {e}")

    def abort_and_return(self, reason: str = 'web_button'):
        node = self._node
        with self._state_lock:
            state = self.robot_state
            if state in (RobotState.ABORTING, RobotState.RUNNING_TO_HOME,
                         RobotState.RETURNING, RobotState.RESUMING,
                         RobotState.EMERGENCY, RobotState.SAFE_EXIT):
                node.get_logger().info(f"ℹ️  이미 {state} — 무시")
                return
            if state == RobotState.CHARGING:
                node.get_logger().warn(
                    "🔋 충전 이동 중 — 중단 및 복귀 불가 (충전 완료 후 자동 대기)")
                return
            # [단계 1] 즉시 파기 (Lock 안, Atomic)
            node.global_path = []
            node.is_mission_active = False
            node.is_waypoint_loop = False
            node.is_waiting = False
            node.path_index = 0
            # YOLO 초기화
            node.yolo_detected = 0
            node.is_forward_path = True
            if state in (RobotState.LOADING, RobotState.UNLOADING):
                self._pending_abort = True
                node.get_logger().info(f"⏳ {state} 대기 중 — 완료 후 귀환")
                return

        self.take_snapshot(abort_reason=reason)
        node.stop_robot()
        if not self.transition(RobotState.ABORTING):
            return
        node.get_logger().info(
            f"🚨 미션 파기 | 원인={reason} | 노드={self.current_node_key}")
        self._retry_count = 0
        self._start_home_path_thread()

    def _start_home_path_thread(self):
        if self._path_calc_thread and self._path_calc_thread.is_alive():
            return
        self._path_calc_thread = threading.Thread(
            target=self._calc_home_path_async, daemon=True)
        self._path_calc_thread.start()

    def _calc_home_path_async(self):
        node = self._node
        with self._state_lock:
            if node.current_pose is None or node.map_data is None:
                node.get_logger().error("❌ 맵/위치 없음 → SAFE_EXIT")
                need_safe_exit = True
            else:
                need_safe_exit = False
                current_pose_copy = node.current_pose[:]
                start_key = self.current_node_key
        if need_safe_exit:
            self.transition(RobotState.SAFE_EXIT)
            return

        self._node.get_logger().info(f"[ReturnMgr] 귀환 시작 키: {start_key}")
        route_keys = []
        current_node_pose = self._key_to_pose(start_key)
        if current_node_pose is not None:
            dist_to_current = sqrt(
                (current_pose_copy[0] - current_node_pose[0])**2 +
                (current_pose_copy[1] - current_node_pose[1])**2)
            if dist_to_current > self._node.stop_tolerance:
                route_keys.append(start_key)
                self._node.get_logger().info(
                    f"[ReturnMgr] {start_key} 미도착(dist={dist_to_current:.2f}m) → 삽입")

        key = self.RETURN_NEXT.get(start_key)
        while key is not None:
            route_keys.append(key)
            key = self.RETURN_NEXT.get(key)

        return_sequence = []
        for k in route_keys:
            pose = self._key_to_pose(k)
            if pose is None:
                self._handle_failure(f'KEY_TO_POSE:{k}')
                return
            return_sequence.append((k, pose))

        if not return_sequence:
            return_sequence = [('standby', node.rooms.get('standby'))]
            if return_sequence[0][1] is None:
                self._handle_failure('NO_STANDBY')
                return

        first_key, first_pose = return_sequence[0]
        node.get_logger().info(f"[A*] Calculating home path: {start_key} → {[k for k in route_keys]}")
        path_grid = node.run_astar(
            node.world_to_grid(current_pose_copy),
            node.world_to_grid(first_pose))
        if path_grid is None:
            node.get_logger().error(f"[A*] Home Path FAILED: {start_key} → {first_key}")
            self._handle_failure('ASTAR_FAILED')
            return
        node.get_logger().info(f"[A*] Home Path Success: {len(path_grid)} waypoints")

        new_path = [node.grid_to_world(p) for p in path_grid]
        now_ns = node.get_clock().now().nanoseconds
        with self._state_lock:
            node.global_path = new_path
            node.path_index = 0
            node.is_initial_turn = True
            node._last_progress_time = now_ns
            node._last_progress_pose = current_pose_copy
            self._return_sequence = return_sequence
            self._return_idx = 0
            self._retry_count = 0
            # 새 경로 설정 → IDLE stop 플래그 초기화
            node._idle_stop_sent = False

        if self.transition(RobotState.RUNNING_TO_HOME):
            node.publish_path_viz()
            node.get_logger().info(
                f"🏠 귀환 시작 | 경로: {[k for k, _ in return_sequence]}")

    def advance_home(self):
        self._return_idx += 1
        if self._return_idx < len(self._return_sequence):
            key, pose = self._return_sequence[self._return_idx]
            self._set_destination_safe(pose, key)
        else:
            self._on_home_arrived()

    def advance_return(self):
        self._return_idx += 1
        if self._return_idx < len(self._return_sequence):
            key, pose = self._return_sequence[self._return_idx]
            self._set_destination_safe(pose, key)
        else:
            self._on_home_arrived()

    def _on_home_arrived(self):
        node = self._node
        node.get_logger().info("🏠 홈(대기노드) 귀환 완료!")
        self.update_node_key('standby')
        self.transition(RobotState.IDLE)
        self._retry_count = 0
        self._return_sequence = []
        self._return_idx = 0
        self._pending_abort = False
        node.is_mission_active = False
        node.is_waypoint_loop = False
        node.mission_sequence = []
        node.mission_idx = 0
        # YOLO 초기화
        node.yolo_detected = 0
        node.is_forward_path = True
        # YOLO 정지 트리거 발행
        trigger_off = Int32(); trigger_off.data = 0
        node.pub_detect_trigger.publish(trigger_off)
        cmd_home = String(); cmd_home.data = ArduinoCmd.HOME
        node.pub_forklift_cmd.publish(cmd_home)
        node.get_logger().info(f"[Serial] Sent HOME command to Arduino → 귀환 완료")
        done_msg_home = Bool(); done_msg_home.data = True
        node.pub_mission_done.publish(done_msg_home)
        # IDLE 도착 → stop 1회 발행 후 더 이상 발행 안 하도록 플래그 초기화
        node._idle_stop_sent = False

    def start_precheck(self, resume_to: str = RobotState.RUNNING_TO_HOME):
        if self.transition(RobotState.RESUMING):
            self._precheck_start = time.time()
            self._precheck_resume_state = resume_to
            # RESUMING 진입 시 stop 플래그 초기화 (새로운 장애물 감지 대기)
            self._node._resuming_stop_sent = False
            self._node.get_logger().info(f"🔍 Pre-Check 시작 → 통과 시 {resume_to}")

    def check_precheck(self) -> bool:
        node = self._node
        elapsed = time.time() - (self._precheck_start or time.time())
        if elapsed >= self.PRECHECK_TIMEOUT:
            node.get_logger().error(f"❌ Pre-Check 타임아웃 ({self.PRECHECK_TIMEOUT}s)")
            self._handle_failure('PRECHECK_TIMEOUT')
            return False
        scan_stamp = getattr(node, 'scan_data_stamp', None)
        if scan_stamp is None or node.scan_data is None:
            node.get_logger().warn("⏳ Pre-Check: LiDAR 데이터 대기 중...")
            if not node._resuming_stop_sent:
                node.stop_robot()
                node._resuming_stop_sent = True
            return False
        now_sec = node.get_clock().now().nanoseconds / 1e9
        scan_age = now_sec - scan_stamp
        if scan_age > self.SCAN_MAX_AGE:
            node.get_logger().warn(f"⏳ Pre-Check: LiDAR 오래됨 ({scan_age:.2f}s) — 대기")
            if not node._resuming_stop_sent:
                node.stop_robot()
                node._resuming_stop_sent = True
            return False
        scan_local = np.array(node.scan_data)
        scan_local[np.isnan(scan_local) | np.isinf(scan_local)] = 3.5
        scan_local[scan_local < 0.15] = 3.5
        n = len(scan_local)
        d = n / 360.0
        f_dist = float(np.min(
            np.concatenate((scan_local[:int(40*d)], scan_local[-int(40*d):]))))
        l_dist = float(np.min(scan_local[int(45*d):int(120*d)]))
        r_dist = float(np.min(scan_local[int(240*d):int(315*d)]))
        is_safe = (f_dist >= self.SAFE_FRONT_DIST and
                   l_dist >= self.SAFE_SIDE_DIST and
                   r_dist >= self.SAFE_SIDE_DIST)
        if is_safe:
            node.get_logger().info(
                f"✅ Pre-Check 통과 | 신선도:{scan_age:.3f}s "
                f"F:{f_dist:.2f} L:{l_dist:.2f} R:{r_dist:.2f}")
            # 통과 시 플래그 초기화 (다음 RESUMING 진입 대비)
            node._resuming_stop_sent = False
            resume_to = self._precheck_resume_state
            if self.transition(resume_to):
                if self._return_sequence and self._return_idx < len(self._return_sequence):
                    key, pose = self._return_sequence[self._return_idx]
                    self._set_destination_safe(pose, key)
                else:
                    self._start_home_path_thread()
            return True
        else:
            node.get_logger().warn(
                f"⚠️  Pre-Check 장애물 | F:{f_dist:.2f} L:{l_dist:.2f} R:{r_dist:.2f}")
            if not node._resuming_stop_sent:
                node.stop_robot()
                node._resuming_stop_sent = True
            return False

    def on_load_done(self) -> bool:
        with self._state_lock:
            pending = self._pending_abort
            if pending:
                self._pending_abort = False
        if pending:
            self._node.get_logger().info("✅ 상차 완료 → pending abort 실행")
            self._node.stop_robot()
            if self.transition(RobotState.ABORTING):
                self._retry_count = 0
                self._start_home_path_thread()
            return True
        return False

    def on_unload_done(self) -> bool:
        with self._state_lock:
            pending = self._pending_abort
            if pending:
                self._pending_abort = False
        if pending:
            self._node.get_logger().info("✅ 하차 완료 → pending abort 실행")
            self._node.stop_robot()
            if self.transition(RobotState.ABORTING):
                self._retry_count = 0
                self._start_home_path_thread()
            return True
        return False

    def _start_safe_exit_loop(self):
        self._safe_exit_active = True
        self._safe_exit_thread = threading.Thread(
            target=self._safe_exit_loop, daemon=True)
        self._safe_exit_thread.start()
        self._node.get_logger().error(
            "🚨 [FSM] SAFE_EXIT 진입 — 수동 리셋(R키)으로만 복구 가능")

    def _safe_exit_loop(self):
        node = self._node
        while self._safe_exit_active:
            try:
                cmd = String(); cmd.data = ArduinoCmd.STOP
                node.pub_forklift_cmd.publish(cmd)
                node.get_logger().warn("[Serial] Sent STOP command to Arduino due to SAFE_EXIT")
            except Exception:
                pass
            time.sleep(self.SAFE_EXIT_INTERVAL)

    def _handle_failure(self, reason: str = ''):
        node = self._node
        self._retry_count += 1
        if self._retry_count <= self.MAX_RETRY:
            node.get_logger().warn(
                f"⚠️  경로 실패 ({self._retry_count}/{self.MAX_RETRY}) "
                f"원인={reason} | 2초 후 재시도")
            threading.Timer(2.0, self._retry_once).start()
        else:
            node.get_logger().error(
                f"❌ {self.MAX_RETRY}회 실패 → SAFE_EXIT | 원인={reason}\n"
                f"   노드={self.current_node_key}")
            self.transition(RobotState.SAFE_EXIT)
            node.stop_robot()

    def _retry_once(self):
        state = self.get_state()
        if state in (RobotState.ABORTING, RobotState.RUNNING_TO_HOME, RobotState.RETURNING):
            if self._path_calc_thread and self._path_calc_thread.is_alive():
                return
            self._start_home_path_thread()

    def manual_reset(self):
        with self._state_lock:
            if self.robot_state != RobotState.SAFE_EXIT:
                self._node.get_logger().info("ℹ️  SAFE_EXIT 상태가 아닙니다.")
                return
        self._safe_exit_active = False
        if self._safe_exit_thread:
            self._safe_exit_thread.join(timeout=2.0)
        self._retry_count = 0
        self._snapshot = None
        self._pending_abort = False
        self._return_sequence = []
        self._return_idx = 0
        self._precheck_start = None
        self._path_calc_thread = None
        node = self._node
        node.stop_robot()
        # SAFE_EXIT 해제 → 플래그 초기화
        node._safe_exit_stop_sent = False
        node._idle_stop_sent = False
        with self._state_lock:
            node.global_path = []
            node.path_index = 0
        self.transition(RobotState.IDLE)
        node.get_logger().info("[System] Manual Reset Triggered. All flags cleared. → IDLE")

    def _set_destination_safe(self, goal_pose: list, node_key: str = ''):
        node = self._node
        need_failure = False
        with self._state_lock:
            if node.current_pose is None or node.map_data is None:
                node.get_logger().error("❌ 맵/위치 없음")
                need_failure = True
            else:
                current_pose_copy = node.current_pose[:]
        if need_failure:
            self._handle_failure('NO_MAP_OR_POSE')
            return
        node.get_logger().info(f"[A*] Calculating path to: {node_key}")
        path_grid = node.run_astar(
            node.world_to_grid(current_pose_copy),
            node.world_to_grid(goal_pose))
        if path_grid is None:
            node.get_logger().error(f"[A*] Path FAILED to: {node_key}")
            self._handle_failure(f'ASTAR_TO_{node_key}')
            return
        node.get_logger().info(f"[A*] Path Success to: {node_key} ({len(path_grid)} pts)")
        new_path = [node.grid_to_world(p) for p in path_grid]
        now_ns = node.get_clock().now().nanoseconds
        with self._state_lock:
            node.global_path = new_path
            node.path_index = 0
            node.is_initial_turn = True
            node._last_progress_time = now_ns
            node._last_progress_pose = current_pose_copy
            self._retry_count = 0
            # 새 경로 설정 → IDLE stop 플래그 초기화
            node._idle_stop_sent = False
        node.publish_path_viz()
        node.get_logger().info(f"📍 귀환 목적지: {node_key}")

    def _key_to_pose(self, key: str) -> Optional[list]:
        node = self._node
        if key == 'standby':  return node.rooms.get('standby')
        if key == 'load':     return node.rooms.get(node.current_load_key)
        if key == 'unload':   return node.rooms.get(node.current_unload_key)
        if key == 'charge':   return node.rooms.get('charge')
        if key == 'wp_final': return node.wp_final_pose
        if key.startswith('wp_'):
            try:
                idx = int(key.split('_')[1]) - 1
                if 0 <= idx < len(node.waypoints):
                    return node.waypoints[idx]
            except (ValueError, IndexError):
                pass
        node.get_logger().error(f"❌ 알 수 없는 키: {key}")
        return None

    def is_going_home(self) -> bool:
        return self.get_state() in (
            RobotState.ABORTING, RobotState.RUNNING_TO_HOME,
            RobotState.RETURNING, RobotState.RESUMING)

    def is_suspended(self) -> bool:
        return self.get_state() in (RobotState.LOADING, RobotState.UNLOADING)


class NodeAStar:
    def __init__(self, parent=None, position=None):
        self.parent = parent; self.position = position
        self.g = 0; self.h = 0; self.f = 0
    def __eq__(self, other): return self.position == other.position
    def __lt__(self, other): return self.f < other.f

class NavigationDBHelper:
    def __init__(self):
        self.node_service = NodeService()

    def get_active_map_id(self):
        result = map_service.get_active_map()
        return result['map_seq'] if result else None

    def load_common_nodes(self):
        map_id = self.get_active_map_id()
        if map_id is None: raise RuntimeError("활성화된 맵이 없습니다!")
        nodes = self.node_service.get_nodes({'map_id': map_id})
        common = {}; waypoints_dict = {}
        for node in nodes:
            node_id = node['node_id']
            x = float(node['node_x_coord']); y = float(node['node_y_coord'])
            if node_id == 'WAIT001': common['standby'] = [x, y]
            if node_id == 'CHRG001': common['charge'] = [x, y]
            if node_id.startswith('NODE') and len(node_id) == 7:
                try: waypoints_dict[int(node_id[4:])] = [x, y]
                except ValueError: pass
        return common, [waypoints_dict[k] for k in sorted(waypoints_dict.keys())]

    def load_rooms(self, unload_zone: int):
        map_id = self.get_active_map_id()
        if map_id is None: raise RuntimeError("활성화된 맵이 없습니다!")
        nodes = self.node_service.get_nodes({'map_id': map_id})
        rooms = {}; waypoints_dict = {}
        for node in nodes:
            node_id = node['node_id']
            x = float(node['node_x_coord']); y = float(node['node_y_coord'])
            if node_id == 'LOAD001': rooms['load_1'] = [x, y]
            if node_id == f'UNLD00{unload_zone}': rooms[f'unload_{unload_zone}'] = [x, y]
            if node_id == 'WAIT001': rooms['standby'] = [x, y]
            if node_id == 'CHRG001': rooms['charge'] = [x, y]
            if node_id.startswith('NODE') and len(node_id) == 7:
                try: waypoints_dict[int(node_id[4:])] = [x, y]
                except ValueError: pass
        if 'load_1' not in rooms: raise RuntimeError("LOAD001 노드가 DB에 없습니다!")
        if f'unload_{unload_zone}' not in rooms: raise RuntimeError(f"UNLD00{unload_zone} 노드가 DB에 없습니다!")
        if 'standby' not in rooms: raise RuntimeError("WAIT001 노드가 DB에 없습니다!")
        if 'charge' not in rooms: raise RuntimeError("CHRG001 노드가 DB에 없습니다!")
        return rooms, map_id, [waypoints_dict[k] for k in sorted(waypoints_dict.keys())]


class IntegratedNavigation(Node):
    def __init__(self):
        super().__init__('integrated_navigation')

        qos_profile = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        map_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        reliable_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=10)
        volatile_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE, depth=1)

        # ── 대기 관련 ──
        self.is_waiting = False
        self.wait_duration = 3.0
        self.wait_start_time = None
        self.current_room_type = None
        self._waiting_stop_sent = False

        # ── 1회 stop 발행 플래그 ──
        # 각 정지 상태에서 cmd_vel=0 을 딱 1번만 발행하고 중단하기 위한 플래그
        self._idle_stop_sent = False        # IDLE + 무경로 상태
        self._safe_exit_stop_sent = False   # SAFE_EXIT 상태
        self._emergency_stop_sent = False   # 비상정지(is_emergency_paused) 상태
        self._resuming_stop_sent = False    # RESUMING precheck 장애물/스캔 만료

        # ── 비상정지 ──
        self.is_emergency_paused = False
        self._emergency_source = None  # ★ 추가


        # ── 경유 순환 주행 ──
        self.is_waypoint_loop = False
        self.wp_loop_idx = 0
        self.wp_final_pose = None
        self.wp_target_yaw = 0.0
        self.wp_final_aligning = False

        # ── 배터리 ──
        self.battery_percent = 100.0
        self.battery_threshold = 30.0
        self.is_charging = False
        self._battery_warn_sent = False

        # ── 주행 설정 ──
        self.lookahead_dist = 0.35
        self.linear_vel = 0.15
        self.stop_tolerance = 0.15

        # ── rooms / 미션 ──
        self.rooms = {}
        self.waypoints = []
        self.current_load_key = 'load_1'
        self.current_unload_key = 'unload_1'
        self.mission_sequence = []
        self.mission_idx = 0
        self.is_mission_active = False

        # ── 상태 관리 ──
        self.map_data = None
        self.map_resolution = 0.05
        self.map_origin = [0.0, 0.0]
        self.map_width = 0
        self.map_height = 0
        self.current_pose = None
        self.current_yaw = 0.0
        self.global_path = []
        self.path_index = 0
        self.is_initial_turn = False
        self.scan_data = None
        self._last_progress_time = None
        self._last_progress_pose = None
        self._stuck_timeout = 5.0
        self.current_zone = 1

        # ── YOLO ──
        self.yolo_detected = 0
        self.is_forward_path = True

        # zone → 하차 색상 매핑
        self.zone_color_map = {1: 'BLUE', 2: 'RED', 3: 'YELLOW'}

        # ── 퍼블리셔 ──
        self.pub_cmd           = self.create_publisher(Twist,  '/cmd_vel', 10)
        self.pub_path          = self.create_publisher(Path,   '/planned_path', 10)
        self.pub_forklift_cmd  = self.create_publisher(String, '/forklift_cmd', 10)
        self.pub_mission_start = self.create_publisher(Bool,   '/mission_start', 10)
        self.pub_mission_done  = self.create_publisher(Bool,   '/mission_done', 10)
        self.pub_emergency_resolve_web     = self.create_publisher(Bool, '/emergency_resolve_web',     reliable_profile)
        self.pub_emergency_resolved = self.create_publisher(Bool, '/emergency_resolved', reliable_profile)
        self.return_home_publisher     = self.create_publisher(Bool, '/return_home', 10)
        self.emergency_resolve_web_publisher = self.create_publisher(Bool, '/emergency_resolve_web', reliable_profile)
        self.pub_carry         = self.create_publisher(Int32,  '/carry', reliable_profile)
        self.pub_unld_sig      = self.create_publisher(String, '/get_unload', volatile_profile)
        self.pub_detect_trigger = self.create_publisher(Int32, '/detect_trigger', volatile_profile)

        # ── 서브스크라이버 ──
        self.sub_map        = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos_profile)
        self.sub_pose       = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.scan_sub       = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile)
        self.sub_battery    = self.create_subscription(BatteryState, '/battery_state', self.battery_callback, 10)
        self.sub_goal_pose  = self.create_subscription(PoseStamped, '/waypoint_drive', self.goal_pose_callback, reliable_profile)
        self.sub_emergency_arduino  = self.create_subscription(Bool, '/emergency_stop_arduino',    self.emergency_arduino_callback,         10)
        self.sub_emergency_web      = self.create_subscription(Bool, '/emergency_stop_web',        self.emergency_web_callback,            reliable_profile)
        self.sub_resolve_arduino    = self.create_subscription(Bool, '/emergency_resolve_arduino', self.emergency_resolve_arduino_callback, 10)
        self.sub_resolve_web        = self.create_subscription(Bool, '/emergency_resolve_web',     self.emergency_resolve_web_callback,     reliable_profile)
        self.sub_forklift_done = self.create_subscription(Bool, '/forklift_done', self.forklift_done_callback, 10)
        self.sub_trigger    = self.create_subscription(Int32, '/carry_id', self.trigger_callback, 10)
        self.sub_carry_done = self.create_subscription(String, '/carry_done', self.carry_done_callback, volatile_profile)
        self.sub_unld_done  = self.create_subscription(String, '/done', self.done_callback, volatile_profile)
        self.sub_return_home = self.create_subscription(Bool, '/return_home', self._return_home_callback, 10)
        self.sub_lim_goal   = self.create_subscription(PoseStamped, '/llm_goal', self.llm_goal_callback, 10)
        self.sub_yolo       = self.create_subscription(Int32, '/slow_detected', self.yolo_callback, volatile_profile)

        self.sub_unld_sig = self.create_subscription(String, '/get_unload', self._unload_task_start_callback, volatile_profile)

        # 비상정지 출처 플래그 초기화
        self._emergency_source = None 

        # 공통 노드 로드
        try:
            nav_db = NavigationDBHelper()
            common, waypoints = nav_db.load_common_nodes()
            self.rooms.update(common)
            self.waypoints = waypoints
            self.get_logger().info(f"공통 노드 로드 완료: {list(common.keys())} | 경유 노드 {len(waypoints)}개")
        except Exception as e:
            self.get_logger().warn(f"공통 노드 로드 실패 (미션 실행 시 로드됨): {e}")

        # ReturnManager 초기화
        self.return_mgr = ReturnManager(self)
        self.scan_data_stamp = None
        if self.waypoints:
            self.return_mgr.build_return_graph(self.waypoints)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.keyboard_thread.start()
        self.get_logger().info("🎮 Ready! '1~3'=구역선택 | 'S'=강제중지 | 'O'=비상정지해제 | 'R'=수동리셋")

    def _unload_task_start_callback(self, msg: String):
        """FSMUnloadNode가 하차 작업 시작할 때 cmd_vel 차단"""
        self.get_logger().info("[Nav] 하차 작업 시작 감지 → cmd_vel 발행 차단")
        self.is_unload_task_active = True
        if self.return_mgr.get_state() == RobotState.UNLOADING:
            return

        # 경로 정리 + 대기 상태 설정 (상차 도착 처리와 동일한 패턴)
        self.global_path = []
        self.is_waiting = True
        self._waiting_stop_sent = False
        self.current_room_type = 'unload'
        self.wait_start_time = self.get_clock().now().nanoseconds

        # MISSION 상태가 아닐 경우(LLM 직접 호출) IDLE→UNLOADING 직접 전환
        cur = self.return_mgr.get_state()
        if cur == RobotState.MISSION:
            self.return_mgr.transition(RobotState.UNLOADING)
        else:
            # IDLE 등 → 직접 전환 (VALID_TRANSITIONS에 추가 필요)
            self.return_mgr.transition(RobotState.UNLOADING)

    # ── YOLO 콜백 ──
    def yolo_callback(self, msg):
        self.yolo_detected = msg.data
        if self.yolo_detected == 1:
            self.get_logger().warn("[YOLO] Person/Obstacle Detected! Slowing down...")

    # ── 배터리 콜백 ──
    def battery_callback(self, msg):
        self.battery_percent = msg.percentage
        if self.battery_percent <= self.battery_threshold and not self.is_charging:
            if self.return_mgr.is_going_home(): return
            if self.return_mgr.get_state() == RobotState.SAFE_EXIT: return
            if not self._battery_warn_sent:
                self.get_logger().warn(
                    f"🔋 배터리 부족 ({self.battery_percent:.1f}%) → 충전 노드로!")
                self._battery_warn_sent = True
            self.is_mission_active = False
            self.is_waypoint_loop = False
            self.is_waiting = False
            self.global_path = []
            self._go_to_charge()
        else:
            self._battery_warn_sent = False

    def _go_to_charge(self):
        if self.return_mgr.is_going_home():
            self.get_logger().warn("⚠️  귀환 중 — 충전 이동 스킵"); return
        if self.map_data is None or self.current_pose is None:
            self.get_logger().error("❌ 맵/위치 정보 없음, 충전 이동 불가"); return
        if 'charge' not in self.rooms:
            self.get_logger().error("❌ 충전 노드 미로드"); return
        self.is_charging = True
        self.return_mgr.transition(RobotState.CHARGING)
        self.get_logger().info("🔌 충전 노드로 이동 시작")
        self.set_next_destination(self.rooms['charge'])

    # ── /waypoint_drive ──
    def goal_pose_callback(self, msg):
        if self.is_mission_active:
            self.get_logger().warn("⚠️  미션 주행 중 — /waypoint_drive 무시"); return
        if self.return_mgr.is_going_home():
            self.get_logger().warn(f"⚠️  귀환 중 — /waypoint_drive 무시"); return
        if self.map_data is None or self.current_pose is None:
            self.get_logger().error("❌ 맵/위치 미수신"); return
        if not self.waypoints:
            self.get_logger().error("❌ 경유 노드 없음"); return
        if self.is_waypoint_loop:
            self.get_logger().info("ℹ️  이미 경유 주행 중"); return
        self.get_logger().info(
            f"📩 /waypoint_drive 수신! x={msg.pose.position.x:.3f} y={msg.pose.position.y:.3f}")
        self.wp_final_pose = [msg.pose.position.x, msg.pose.position.y]
        qz = msg.pose.orientation.z; qw = msg.pose.orientation.w
        self.wp_target_yaw = atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
        self.is_waypoint_loop = True
        self.is_forward_path = True
        self.yolo_detected = 0
        self.wp_loop_idx = 0
        self.wp_final_aligning = False
        self._idle_stop_sent = False   # 새 주행 시작 → IDLE stop 플래그 초기화
        self.return_mgr.transition(RobotState.WAYPOINT)
        self.return_mgr.update_node_key('wp_1')
        self.set_next_destination(self.waypoints[self.wp_loop_idx])

    # ── 비상정지 ──
    def emergency_arduino_callback(self, msg):
        """아두이노발 비상정지"""
        if not msg.data or self.is_emergency_paused:
            return
        self.get_logger().warn("[Arduino] Emergency Stop → SAFE_EXIT")
        self._emergency_source = 'arduino'
        self.is_emergency_paused = True
        self.stop_robot()
        if self.return_mgr.get_state() != RobotState.SAFE_EXIT:
            self.return_mgr.transition(RobotState.EMERGENCY)
            self.return_mgr.transition(RobotState.SAFE_EXIT)

    def emergency_web_callback(self, msg):
        """웹발 비상정지"""
        if not msg.data or self.is_emergency_paused:
            return
        self.get_logger().warn("[Web] Emergency Stop → SAFE_EXIT")
        self._emergency_source = 'web'
        self.is_emergency_paused = True
        self.stop_robot()
        if self.return_mgr.get_state() != RobotState.SAFE_EXIT:
            self.return_mgr.transition(RobotState.EMERGENCY)
            self.return_mgr.transition(RobotState.SAFE_EXIT)

    def emergency_resolve_arduino_callback(self, msg):
        """아두이노 버튼 해제 — 아두이노발 비상정지만 해제 가능"""
        if not msg.data or not self.is_emergency_paused:
            return
        if self._emergency_source != 'arduino':
            self.get_logger().warn("[Arduino] 웹 비상정지 상태 — 아두이노 해제 무시")
            return
        self.get_logger().info("[Arduino] Emergency Resolve")
        self._resolve_emergency()

    def emergency_resolve_web_callback(self, msg):
        """웹 해제 — 웹발 비상정지만 해제 가능"""
        if not msg.data or not self.is_emergency_paused:
            return
        if self._emergency_source != 'web':
            self.get_logger().warn("[Web] 아두이노 비상정지 상태 — 웹 해제 무시")
            return
        self.get_logger().info("[Web] Emergency Resolve")
        self._resolve_emergency()

    def _resolve_emergency(self):
        """공통 해제 로직"""
        if self.return_mgr.get_state() == RobotState.SAFE_EXIT:
            self.return_mgr._safe_exit_active = False
            t = self.return_mgr._safe_exit_thread
            if t and t.is_alive():
                t.join(timeout=2.0)
            self.return_mgr.transition(RobotState.IDLE)
            self.is_emergency_paused = False
            self._emergency_source = None
            done_msg = Bool(); done_msg.data = True
            self.pub_emergency_resolved.publish(done_msg)
            self.get_logger().info("✅ 비상정지 해제 완료")

            # ★ 해제 완료 토픽 발행 → drive_bp가 구독해서 소켓 전송
            done_msg = Bool(); done_msg.data = True
            self.pub_emergency_resolved.publish(done_msg)

            if self.global_path:
                self.is_initial_turn = True
                self._last_progress_time = self.get_clock().now().nanoseconds
                self._last_progress_pose = self.current_pose[:]
            elif self.is_mission_active and self.mission_sequence:
                self.return_mgr.transition(RobotState.MISSION)
                self.return_mgr.start_precheck(RobotState.RUNNING_TO_HOME)
            elif self.return_mgr._return_sequence:
                self.return_mgr.start_precheck(RobotState.RUNNING_TO_HOME)
        else:
            self.is_emergency_paused = False
            self._emergency_source = None

    # ── 하차 완료 ──
    def done_callback(self, msg):
        self.get_logger().info(f"📩 /done 수신: '{msg.data}'")
        if self.return_mgr.on_unload_done():
            self.is_waiting = False; return
        if self.is_waiting and self.current_room_type == 'unload':
            self.is_waiting = False
            if self.is_mission_active and self.mission_sequence:
                self.return_mgr.transition(RobotState.MISSION)
                self.get_logger().info("✅ 하차 완료! 다음 목적지로 출발합니다.")
                self._advance_mission()
            else:
                self.return_mgr.transition(RobotState.IDLE)
                self.get_logger().info("✅ [LLM] 하차 완료! IDLE 복귀.")

    # ── forklift_done (standby 전용) ──
    def forklift_done_callback(self, msg):
        if msg.data and self.is_waiting and self.current_room_type == 'standby':
            self.is_waiting = False
            if self.return_mgr.get_state() != RobotState.IDLE:
                self.return_mgr.transition(RobotState.IDLE)
            self.get_logger().info("✅ 대기 완료(forklift_done)! 다음 목적지로 출발합니다.")
            self._advance_mission()

    # ── 상차 완료 ──
    def carry_done_callback(self, msg):
        self.get_logger().info(
            f"📩 /carry_done 수신: {msg.data} | is_waiting={self.is_waiting} | room={self.current_room_type}")
        if self.return_mgr.on_load_done():
            self.is_waiting = False; return
        if msg.data and self.is_waiting and self.current_room_type == 'load':
            self.is_waiting = False
            self.return_mgr.transition(RobotState.MISSION)
            self.get_logger().info("✅ 상차 완료! 다음 목적지로 출발합니다.")
            self._advance_mission()

    def _advance_mission(self):
        # 경유 주행 모드
        if self.is_waypoint_loop:
            if not self.waypoints:
                self.is_waypoint_loop = False; return
            next_idx = self.wp_loop_idx + 1
            if next_idx < len(self.waypoints):
                self.wp_loop_idx = next_idx
                self.return_mgr.update_node_key(f'wp_{self.wp_loop_idx+1}')
                self.get_logger().info(f"🔀 경유: {self.wp_loop_idx+1}/{len(self.waypoints)}번 노드로 이동")
                self.set_next_destination(self.waypoints[self.wp_loop_idx])
            else:
                self.return_mgr.update_node_key('wp_final')
                self.get_logger().info(f"🏁 경유 완료 → 최종 목적지: {self.wp_final_pose}")
                self.set_next_destination(self.wp_final_pose)
            return
        # 충전 완료
        if self.is_charging:
            self.get_logger().info("🔌 충전 노드 도착! 충전 대기 중...")
            self.is_charging = False
            self.return_mgr.transition(RobotState.IDLE); return
        if not self.is_mission_active: return
        self.mission_idx += 1
        if self.mission_idx < len(self.mission_sequence):
            self.set_next_destination(self.mission_sequence[self.mission_idx])
        else:
            self.get_logger().info("🏁 MISSION COMPLETE!")
            self.is_mission_active = False
            self.return_mgr.transition(RobotState.IDLE)
            # YOLO 정지 트리거
            trigger_off = Int32(); trigger_off.data = 0
            self.pub_detect_trigger.publish(trigger_off)
            done_msg = Bool(); done_msg.data = True
            self.pub_mission_done.publish(done_msg)
            self.global_path = []

    def _full_reset(self):
        self.return_mgr.manual_reset()
        self.is_mission_active = False
        self.is_waiting = False
        self.is_waypoint_loop = False
        self.is_charging = False
        self.is_emergency_paused = False
        self._emergency_source = None  # ★ 추가
        self.is_initial_turn = False
        self.wp_final_aligning = False
        self._waiting_stop_sent = False
        self._battery_warn_sent = False
        self._last_progress_time = None
        self._last_progress_pose = None
        self.scan_data_stamp = None
        self.yolo_detected = 0
        self.is_forward_path = True
        # 1회 stop 플래그 전체 초기화
        self._idle_stop_sent = False
        self._safe_exit_stop_sent = False
        self._emergency_stop_sent = False
        self._resuming_stop_sent = False
        self.stop_robot()
        self.global_path = []
        self.path_index = 0
        self.mission_sequence = []
        self.mission_idx = 0
        self.current_room_type = None
        # YOLO 정지 트리거 발행
        trigger_off = Int32(); trigger_off.data = 0
        self.pub_detect_trigger.publish(trigger_off)
        self.get_logger().info("[System] Full Reset Complete. All mission/nav flags cleared.")

    def _return_home_callback(self, msg):
        if msg.data:
            self.get_logger().info("[Web Input] Abort & Return Requested")
            self.return_mgr.abort_and_return(reason='web_button')

    def llm_goal_callback(self, msg):
        if self.is_mission_active:
            self.get_logger().warn("미션 주행 중"); return
        if self.map_data is None or self.current_pose is None:
            self.get_logger().error("맵/위치 정보 미수신"); return
        x = msg.pose.position.x; y = msg.pose.position.y
        qz = msg.pose.orientation.z; qw = msg.pose.orientation.w
        self.wp_target_yaw = atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
        self.get_logger().info(f"LLM 노드 이동 → x={x:.3f}, y={y:.3f}, yaw={self.wp_target_yaw:.2f}rad")
        self.wp_final_aligning = True
        self.set_next_destination([x, y])

    # ── 키보드 ──
    def keyboard_listener(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while True:
                ch = sys.stdin.read(1)
                if ch in ('1', '2', '3'):
                    self.trigger_mission(unload_zone=int(ch))
                elif ch in ('s', 'S'):
                    self.get_logger().info("🛑 강제 중지 명령 수신.")
                    self.is_mission_active = False
                    self.is_waiting = False
                    self.is_waypoint_loop = False
                    self.is_charging = False
                    self.global_path = []
                    self.stop_robot()
                    # YOLO 정지
                    trigger_off = Int32(); trigger_off.data = 0
                    self.pub_detect_trigger.publish(trigger_off)
                    self.yolo_detected = 0
                    self.is_forward_path = True
                    cur = self.return_mgr.get_state()
                    if cur in (RobotState.LOADING, RobotState.UNLOADING):
                        self.return_mgr.transition(RobotState.ABORTING)
                        self.return_mgr.transition(RobotState.SAFE_EXIT)
                        self.return_mgr.manual_reset()
                    elif cur in (RobotState.ABORTING, RobotState.RESUMING):
                        self.return_mgr.transition(RobotState.SAFE_EXIT)
                        self.return_mgr.manual_reset()
                    else:
                        self.return_mgr.transition(RobotState.IDLE)
                        self._idle_stop_sent = False  # IDLE 전환 시 플래그 초기화
                    done_msg = Bool(); done_msg.data = True
                    self.pub_mission_done.publish(done_msg)

                elif ch in ('o', 'O'):
                    if self.is_emergency_paused:
                        if self._emergency_source == 'web':
                            resolve_msg = Bool(); resolve_msg.data = True
                            self.pub_emergency_resolve_web.publish(resolve_msg)
                            self.get_logger().info("✅ [O키] 웹 비상정지 해제.")
                        else:
                            self.get_logger().warn("ℹ️  아두이노 비상정지 상태 — O키로 해제 불가 (아두이노 버튼 사용)")
                    else:
                        self.get_logger().info("ℹ️  현재 비상정지 상태가 아닙니다.")

                elif ch in ('r', 'R'):
                    self.get_logger().info("🔄 수동 리셋 명령 수신.")
                    self._full_reset()
                elif ch == '\x03':
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def trigger_callback(self, msg):
        unload_map = {0: 1, 1: 2, 2: 3}
        if msg.data not in unload_map:
            self.get_logger().error(f"❌ /carry_id 유효하지 않은 값: {msg.data}"); return
        unload_zone = unload_map[msg.data]
        self.get_logger().info(f"🔔 /carry_id 수신: {msg.data} → UNLD00{unload_zone} 미션 시작")
        self.trigger_mission(unload_zone=unload_zone)

    def trigger_mission(self, unload_zone: int = 1):
        if self.is_mission_active:
            self.get_logger().warn("⚠️  미션이 이미 진행 중입니다!"); return
        self.is_forward_path = True
        self.yolo_detected = 0
        if self.return_mgr.is_going_home():
            self.get_logger().warn(f"⚠️  귀환 중 — 미션 시작 차단"); return
        if self.is_charging:
            self.get_logger().warn("⚠️  충전 이동 중 — 미션 시작 차단"); return
        if self.map_data is None or self.current_pose is None:
            self.get_logger().error("❌ 맵 또는 위치 정보가 준비되지 않았습니다!"); return
        self.current_zone = unload_zone
        try:
            nav_db = NavigationDBHelper()
            self.rooms, active_map_id, self.waypoints = nav_db.load_rooms(unload_zone)
            self.get_logger().info(
                f"✅ DB 로드 완료 (map_id={active_map_id})\n"
                f"   상차: {self.rooms['load_1']}\n"
                f"   하차: {self.rooms[f'unload_{unload_zone}']}\n"
                f"   대기: {self.rooms['standby']}")
        except Exception as e:
            self.get_logger().error(f"❌ DB 로드 실패: {e}"); return
        self.current_load_key = 'load_1'
        self.current_unload_key = f'unload_{unload_zone}'
        self.return_mgr.build_return_graph(self.waypoints)
        self.return_mgr.update_node_key('load')
        self.mission_sequence = []
        wp_fwd = self.waypoints
        wp_rev = self.waypoints[::-1]
        for _ in range(5):
            self.mission_sequence.append(self.rooms[self.current_load_key])
            self.mission_sequence.extend(wp_fwd)
            self.mission_sequence.append(self.rooms[self.current_unload_key])
            if _ < 4:
                self.mission_sequence.extend(wp_rev)
        self.mission_sequence.append(self.rooms['standby'])
        self.get_logger().info(f"🚀 미션 시작! [UNLD00{unload_zone}] 상차→하차 5회 왕복.")
        start_msg = Bool(); start_msg.data = True
        self.pub_mission_start.publish(start_msg)
        self.is_mission_active = True
        self.mission_idx = 0
        self._idle_stop_sent = False   # 미션 시작 → IDLE stop 플래그 초기화
        self.return_mgr.transition(RobotState.MISSION)
        self.set_next_destination(self.mission_sequence[0])

    def set_next_destination(self, goal_pose):
        self.get_logger().info(f"📍 다음 목표: {goal_pose}")
        if self.return_mgr.is_going_home():
            self.get_logger().warn("⚠️  귀환 중 일반 경로 탐색 요청 차단"); return
        drive_msg = Bool(); drive_msg.data = True
        self.pub_mission_start.publish(drive_msg)
        path_grid = self.run_astar(
            self.world_to_grid(self.current_pose),
            self.world_to_grid(goal_pose))
        if path_grid:
            self.global_path = [self.grid_to_world(p) for p in path_grid]
            self.path_index = 0
            self.is_initial_turn = True
            self._last_progress_time = self.get_clock().now().nanoseconds
            self._last_progress_pose = self.current_pose[:]
            self._idle_stop_sent = False   # 새 경로 설정 → IDLE stop 플래그 초기화
            self.publish_path_viz()
        else:
            self.get_logger().error("❌ 경로를 찾을 수 없습니다!")
            self.is_mission_active = False

    def run_astar(self, start, end):
        if self.map_data is None: return None
        if not (0 <= start[0] < self.map_height and 0 <= start[1] < self.map_width): return None
        if not (0 <= end[0]   < self.map_height and 0 <= end[1]   < self.map_width): return None
        start_node = NodeAStar(None, start)
        end_node   = NodeAStar(None, end)
        open_list = []; heapq.heappush(open_list, start_node)
        visited = set()
        moves = [(0,1,1.0),(0,-1,1.0),(1,0,1.0),(-1,0,1.0),
                 (1,1,1.41),(1,-1,1.41),(-1,1,1.41),(-1,-1,1.41)]
        safety_margin = 2.6; preferred_margin = 5.8; obstacle_threshold = 80
        while open_list:
            cur = heapq.heappop(open_list)
            if cur.position in visited: continue
            visited.add(cur.position)
            if cur.position == end_node.position:
                path = []
                c = cur
                while c: path.append(c.position); c = c.parent
                full = path[::-1]
                smoothed = full[::3]
                if full[-1] not in smoothed: smoothed.append(full[-1])
                return smoothed
            for dy, dx, cost in moves:
                ny, nx = cur.position[0]+dy, cur.position[1]+dx
                if not (0 <= ny < self.map_height and 0 <= nx < self.map_width): continue
                if self.map_data[ny][nx] > obstacle_threshold or self.map_data[ny][nx] == -1: continue
                penalty = 0; too_close = False
                for r in range(1, int(preferred_margin) + 1):
                    for cy, cx in [(ny+r,nx),(ny-r,nx),(ny,nx+r),(ny,nx-r)]:
                        if 0 <= cy < self.map_height and 0 <= cx < self.map_width:
                            if self.map_data[cy][cx] > obstacle_threshold:
                                if r <= safety_margin: too_close = True; break
                                penalty += (preferred_margin - r) * 12
                    if too_close: break
                if too_close: continue
                nn = NodeAStar(cur, (ny, nx))
                nn.g = cur.g + cost + penalty
                nn.h = sqrt((ny-end[0])**2 + (nx-end[1])**2)
                nn.f = nn.g + nn.h
                heapq.heappush(open_list, nn)
        return None

    def control_loop(self):
        rm_state = self.return_mgr.get_state()

        # ── SAFE_EXIT: cmd_vel=0 을 딱 1회만 발행 후 중단 ──
        if rm_state == RobotState.SAFE_EXIT:
            if not self._safe_exit_stop_sent:
                self.stop_robot()
                self._safe_exit_stop_sent = True
            return

        # ── IDLE + 무경로: cmd_vel=0 을 딱 1회만 발행 후 중단 ──
        if rm_state == RobotState.IDLE and not self.is_mission_active \
                and not self.is_waypoint_loop and not self.is_charging:
            if not self.global_path and not self.wp_final_aligning:
                if not self._idle_stop_sent:
                    self.stop_robot()
                    self._idle_stop_sent = True
                return

        # ── RESUMING: Pre-Check ──
        if rm_state == RobotState.RESUMING:
            self.return_mgr.check_precheck(); return

        # ── 비상정지: cmd_vel=0 을 딱 1회만 발행 후 중단 ──
        if self.is_emergency_paused:
            if not self._emergency_stop_sent:
                self.stop_robot()
                self._emergency_stop_sent = True
            return

        if self.scan_data is None or self.current_pose is None:
            return

        # yaw 정렬
        if self.wp_final_aligning:
            yaw_err = self.wp_target_yaw - self.current_yaw
            while yaw_err >  pi: yaw_err -= 2*pi
            while yaw_err < -pi: yaw_err += 2*pi
            if abs(yaw_err) > 0.05:
                cmd = Twist()
                cmd.angular.z = 0.3 if yaw_err > 0 else -0.3
                self.pub_cmd.publish(cmd); return
            else:
                self.stop_robot()
                self.wp_final_aligning = False
                self.wp_final_pose = None
                self.return_mgr.update_node_key('standby')
                self.get_logger().info("✅ 최종 방향 정렬 완료! 정지."); return

        # 대기 중
        if self.is_waiting:
            if not self._waiting_stop_sent:
                self.stop_robot()
                self._waiting_stop_sent = True
            if self.current_room_type == 'load':
                elapsed = (self.get_clock().now().nanoseconds - self.wait_start_time) / 1e9
                if int(elapsed) != int(elapsed - 0.1):
                    self.get_logger().info(f"🔼 상차 대기 중... ({elapsed:.0f}초)")
                return
            if self.current_room_type == 'unload':
                elapsed = (self.get_clock().now().nanoseconds - self.wait_start_time) / 1e9
                if int(elapsed) != int(elapsed - 0.1):
                    self.get_logger().info(f"🔽 하차 대기 중... ({elapsed:.0f}초)")
                return
            if self.current_room_type == 'standby':
                elapsed = (self.get_clock().now().nanoseconds - self.wait_start_time) / 1e9
                remaining = self.wait_duration - elapsed
                if int(elapsed) != int(elapsed - 0.1):
                    self.get_logger().info(f"🅿️  대기 중... ({remaining:.1f}초 남음)")
                if elapsed >= self.wait_duration:
                    self.is_waiting = False
                    if self.return_mgr.get_state() != RobotState.IDLE:
                        self.return_mgr.transition(RobotState.IDLE)
                    self.get_logger().info("✅ 대기 완료! 다음 목적지로 출발합니다.")
                    self._advance_mission()
            return

        if not self.global_path: return

        # LiDAR 전처리
        ranges = np.array(self.scan_data)
        ranges[np.isnan(ranges) | np.isinf(ranges)] = 3.5
        ranges[ranges < 0.15] = 3.5
        num_points = len(ranges)
        idx_per_deg = num_points / 360.0
        f_idx = int(40 * idx_per_deg)
        front_ranges = np.concatenate((ranges[:f_idx], ranges[-f_idx:]))
        left_ranges  = ranges[int(45*idx_per_deg):int(120*idx_per_deg)]
        right_ranges = ranges[int(240*idx_per_deg):int(315*idx_per_deg)]
        back_ranges  = ranges[int(150*idx_per_deg):int(210*idx_per_deg)]
        fl_ranges    = ranges[int(20*idx_per_deg):int(60*idx_per_deg)]
        fr_ranges    = ranges[int(300*idx_per_deg):int(340*idx_per_deg)]
        f_dist  = np.min(front_ranges)
        l_dist  = np.min(left_ranges)
        r_dist  = np.min(right_ranges)
        b_dist  = np.min(back_ranges)
        fl_dist = np.min(fl_ranges)
        fr_dist = np.min(fr_ranges)

        # 도착 판정
        if not self.global_path: return
        final_goal = self.global_path[-1]
        dist_to_goal = sqrt((final_goal[0]-self.current_pose[0])**2 +
                            (final_goal[1]-self.current_pose[1])**2)

        if dist_to_goal < self.stop_tolerance:
            self.stop_robot()
            cmd_msg = String()

            # 귀환 중 도착
            if self.return_mgr.is_going_home():
                self.global_path = []
                if rm_state == RobotState.RUNNING_TO_HOME:
                    self.return_mgr.update_node_key(
                        self.return_mgr._return_sequence[self.return_mgr._return_idx][0]
                        if self.return_mgr._return_sequence else 'standby')
                    self.return_mgr.advance_home()
                else:
                    self.return_mgr.advance_return()
                return

            # 경유 주행 도착
            if self.is_waypoint_loop:
                if not self.global_path: return
                is_final = (
                    self.wp_final_pose is not None and
                    self.wp_loop_idx >= len(self.waypoints) - 1 and
                    self.global_path[-1] == self.grid_to_world(self.world_to_grid(self.wp_final_pose)))
                if is_final or (
                    self.wp_final_pose is not None and
                    abs(self.current_pose[0] - self.wp_final_pose[0]) < self.stop_tolerance and
                    abs(self.current_pose[1] - self.wp_final_pose[1]) < self.stop_tolerance):
                    self.stop_robot()
                    self.global_path = []
                    self.wp_final_aligning = True
                    self.get_logger().info(f"🎯 최종 목적지 도착! yaw={self.wp_target_yaw:.2f}rad 정렬 시작")
                    self.is_waypoint_loop = False
                    self.return_mgr.transition(RobotState.IDLE)
                    return
                wp_idx = self.wp_loop_idx
                self.return_mgr.update_node_key(f'wp_{wp_idx+1}')
                # YOLO 트리거
                trigger_msg = Int32()
                if wp_idx == 0:
                    trigger_msg.data = 1
                    self.pub_detect_trigger.publish(trigger_msg)
                    self.get_logger().info("📡 [Trigger] 1번 WP: YOLO 시작")
                elif wp_idx == 1:
                    trigger_msg.data = 0
                    self.pub_detect_trigger.publish(trigger_msg)
                    self.get_logger().info("📡 [Trigger] 2번 WP: YOLO 중지")
                elif wp_idx == 3:
                    self.yolo_detected = 0
                    self.get_logger().warn("🚀 [Trigger] 4번 WP: 감속 해제")
                self.get_logger().info(f"🔀 경유 통과! ({wp_idx+1}/{len(self.waypoints)})")
                self.wp_loop_idx += 1
                self.global_path = []
                if self.wp_loop_idx < len(self.waypoints):
                    self.set_next_destination(self.waypoints[self.wp_loop_idx])
                else:
                    self.set_next_destination(self.wp_final_pose)
                return

            # 충전 도착
            if self.is_charging:
                self.get_logger().info("🔌 충전 노드 도착!")
                self.is_charging = False
                self.is_waiting = False
                self.global_path = []
                self.return_mgr.update_node_key('charge')
                self.return_mgr.transition(RobotState.IDLE)
                return

            if not self.is_mission_active or not self.mission_sequence:
                self.get_logger().warn("⚠️  mission_sequence 없음 — 경로 초기화")
                self.global_path = []; return

            current_goal = self.mission_sequence[self.mission_idx]

            # 경유 노드 도착
            wp_idx = -1
            for i, wp in enumerate(self.waypoints):
                if abs(current_goal[0]-wp[0]) < 0.15 and abs(current_goal[1]-wp[1]) < 0.15:
                    wp_idx = i; break
            if wp_idx >= 0:
                self.return_mgr.update_node_key(f'wp_{wp_idx+1}')
                is_valid_yolo = (
                    (self.is_mission_active and self.is_forward_path) or
                    self.is_waypoint_loop or
                    self.return_mgr.get_state() == RobotState.WAYPOINT)
                if is_valid_yolo:
                    trigger_msg = Int32()
                    if wp_idx == 0:
                        trigger_msg.data = 1
                        self.pub_detect_trigger.publish(trigger_msg)
                        self.get_logger().info("📡 [Trigger] 1번 WP: YOLO 탐지 시작")
                    elif wp_idx == 1:
                        trigger_msg.data = 0
                        self.pub_detect_trigger.publish(trigger_msg)
                        self.get_logger().info("📡 [Trigger] 2번 WP: YOLO 탐지 중지")
                    elif wp_idx == 3:
                        self.yolo_detected = 0
                        self.get_logger().warn("🚀 [Trigger] 4번 WP: 감속 해제")
                self.get_logger().info("🔀 경유 노드 통과! 다음으로 진행")
                self.global_path = []
                self._advance_mission()
                return

            # 상차 도착
            if current_goal == self.rooms[self.current_load_key]:
                self.is_forward_path = True
                self.get_logger().info(f"📦 상차 도착! /carry_done 대기 중...")
                self.current_room_type = 'load'
                self.return_mgr.update_node_key('load')
                self.return_mgr.transition(RobotState.LOADING)
                cmd_msg.data = 'LOAD'
                self.pub_forklift_cmd.publish(cmd_msg)
                self.get_logger().info("[Serial] Sent LOAD command to Arduino")
                carry_msg = Int32(); carry_msg.data = self.current_zone - 1
                self.pub_carry.publish(carry_msg)
                self.is_waiting = True
                self._waiting_stop_sent = False
                self.wait_start_time = self.get_clock().now().nanoseconds
                self.global_path = []; return

            # 하차 도착
            elif current_goal == self.rooms[self.current_unload_key]:
                self.is_forward_path = False
                self.current_room_type = 'unload'
                self.return_mgr.update_node_key('unload')
                self.return_mgr.transition(RobotState.UNLOADING)
                color = self.zone_color_map.get(self.current_zone, 'BLUE')
                self.get_logger().info(f"📦 하차 도착! /get_unload → {color}")
                cmd_msg.data = 'UNLD'
                self.pub_forklift_cmd.publish(cmd_msg)
                self.get_logger().info("[Serial] Sent UNLD command to Arduino")
                unld_msg = String(); unld_msg.data = color
                self.pub_unld_sig.publish(unld_msg)

            # 대기실 도착
            elif current_goal == self.rooms['standby']:
                self.current_room_type = 'standby'
                self.return_mgr.update_node_key('standby')
                self.return_mgr.transition(RobotState.IDLE)
                self.get_logger().info("🅿️  대기실 도착!")

            self.is_waiting = True
            self._waiting_stop_sent = False
            self.wait_start_time = self.get_clock().now().nanoseconds
            self.global_path = []; return

        # 초기 제자리 회전
        if self.is_initial_turn:
            target_idx = min(len(self.global_path)-1, 5)
            target_x, target_y = self.global_path[target_idx]
            alpha_init = atan2(target_y-self.current_pose[1],
                               target_x-self.current_pose[0]) - self.current_yaw
            while alpha_init >  pi: alpha_init -= 2*pi
            while alpha_init < -pi: alpha_init += 2*pi
            turn_margin = 0.25
            cmd = Twist()
            if (f_dist < turn_margin) or \
               (alpha_init > 0 and l_dist < turn_margin) or \
               (alpha_init < 0 and r_dist < turn_margin):
                cmd.linear.x = -0.1 if b_dist > 0.3 else 0.0
                cmd.angular.z = 0.0
                self.pub_cmd.publish(cmd); return
            elif b_dist < turn_margin:
                cmd.linear.x = 0.1 if f_dist > 0.3 else 0.0
                cmd.angular.z = 0.0
                self.pub_cmd.publish(cmd); return
            if abs(alpha_init) > 0.15:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.4 if alpha_init > 0 else -0.4
                self.pub_cmd.publish(cmd); return
            else:
                self.get_logger().info("★★★ 방향 정렬 완료! ★★★")
                self.is_initial_turn = False
                self._last_progress_time = self.get_clock().now().nanoseconds
                self._last_progress_pose = self.current_pose[:]

        # stuck 감지
        if self._last_progress_time is not None and self._last_progress_pose is not None:
            now = self.get_clock().now().nanoseconds
            elapsed_stuck = (now - self._last_progress_time) / 1e9
            moved = sqrt((self.current_pose[0]-self._last_progress_pose[0])**2 +
                         (self.current_pose[1]-self._last_progress_pose[1])**2)
            if moved > 0.05:
                self._last_progress_time = now
                self._last_progress_pose = self.current_pose[:]
            elif elapsed_stuck >= self._stuck_timeout:
                self.get_logger().warn(f"⚠️  {self._stuck_timeout:.0f}초 동안 경로 진행 없음 → 재설정!")
                self._last_progress_time = None
                if not self.global_path: return
                goal = self.global_path[-1][:]
                self.global_path = []
                if self.return_mgr.is_going_home():
                    self.return_mgr._set_destination_safe(goal, 'stuck_retry')
                else:
                    self.set_next_destination(goal)
                return

        # Pure Pursuit
        target_x, target_y = self.global_path[-1]
        for i in range(self.path_index, len(self.global_path)):
            px, py = self.global_path[i]
            dist = sqrt((px-self.current_pose[0])**2 + (py-self.current_pose[1])**2)
            if dist >= self.lookahead_dist:
                target_x, target_y = px, py
                self.path_index = i; break
        alpha = atan2(target_y-self.current_pose[1],
                      target_x-self.current_pose[0]) - self.current_yaw
        while alpha >  pi: alpha -= 2*pi
        while alpha < -pi: alpha += 2*pi

        yolo_factor = 0.6 if self.yolo_detected == 1 else 1.0

        cmd = Twist()
        if f_dist < 0.22 or l_dist < 0.18 or r_dist < 0.18:
            action = "stop"
        elif f_dist < 0.27:
            action = "go_back"
        elif fl_dist < 0.25 or fr_dist < 0.25:
            action = "turn_avoid"
            cmd.linear.x = 0.05
            avoid_dir = -0.25 if fl_dist < fr_dist else 0.25
            cmd.angular.z = avoid_dir + (0.3 * sin(alpha))
        elif f_dist < 0.35:
            action = "turn_avoid"
            cmd.linear.x = 0.05
            avoid_dir = 0.22 if l_dist >= r_dist else -0.22
            cmd.angular.z = avoid_dir + (0.3 * sin(alpha))
        else:
            action = "go_forward"

        if action == "go_forward":
            turn_factor = max(0.4, 1.0 - abs(alpha) / pi)
            wall_factor = 0.8 if l_dist < 0.35 or r_dist < 0.35 else 1.0
            spd = self.linear_vel * wall_factor * turn_factor * yolo_factor
            cmd.linear.x = spd
            steering = (3.0 * sin(alpha)) / self.lookahead_dist
            if l_dist < 0.35: steering -= 0.6 * (0.35 - l_dist)
            if r_dist < 0.35: steering += 0.6 * (0.35 - r_dist)
            cmd.angular.z = cmd.linear.x * steering
        elif action == "go_back":
            cmd.linear.x = (-0.15 if b_dist > 0.2 else 0.0) * yolo_factor
            cmd.angular.z = 0.0
        elif action == "stop":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.pub_cmd.publish(cmd)

    def scan_callback(self, msg):
        r = np.array(msg.ranges)
        self.scan_data = np.where(np.isinf(r), 3.5, np.where(np.isnan(r), 3.5, r))
        self.scan_data_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

    def map_callback(self, msg):
        self.map_resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        self.map_data = np.array(msg.data).reshape((self.map_height, self.map_width))

    def pose_callback(self, msg):
        self.current_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        q = msg.pose.pose.orientation
        self.current_yaw = atan2(2.0*(q.w*q.z + q.x*q.y), 1.0-2.0*(q.y*q.y + q.z*q.z))

    def world_to_grid(self, world):
        return (int((world[1]-self.map_origin[1])/self.map_resolution),
                int((world[0]-self.map_origin[0])/self.map_resolution))

    def grid_to_world(self, grid):
        return [(grid[1]*self.map_resolution)+self.map_origin[0],
                (grid[0]*self.map_resolution)+self.map_origin[1]]

    def publish_path_viz(self):
        msg = Path(); msg.header.frame_id = 'map'
        for p in self.global_path:
            ps = PoseStamped()
            ps.pose.position.x, ps.pose.position.y = p[0], p[1]
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    def stop_robot(self):
        self.pub_cmd.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = IntegratedNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()