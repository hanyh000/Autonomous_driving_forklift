import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition
import time
from geometry_msgs.msg import PoseWithCovarianceStamped

class MapManager(Node):
    def __init__(self):
        self._ctx = rclpy.Context()
        rclpy.init(context=self._ctx)
        super().__init__('map_manager_client', context=self._ctx)
        self.ordered_nodes = [
            '/controller_server', '/planner_server', '/smoother_server',
            '/behavior_server', '/bt_navigator', '/waypoint_follower', '/velocity_smoother'
        ]

    def _change_state(self, node_name, transition_id):
        client = self.create_client(ChangeState, f'{node_name}/change_state')
        if not client.wait_for_service(timeout_sec=2.0):
            print(f"[{node_name}] 서비스를 찾을 수 없습니다.")
            return False

        req = ChangeState.Request()
        req.transition.id = transition_id

        future = client.call_async(req)
        executor = rclpy.executors.SingleThreadedExecutor(context=self._ctx)
        executor.add_node(self)
        executor.spin_until_future_complete(future, timeout_sec=30.0)
        executor.remove_node(self)
        return future.result() is not None

    def _get_state(self, node_name):
        client = self.create_client(GetState, f'{node_name}/get_state')
        if not client.wait_for_service(timeout_sec=1.0):
            return "unknown"

        req = GetState.Request()
        future = client.call_async(req)
        executor = rclpy.executors.SingleThreadedExecutor(context=self._ctx)
        executor.add_node(self)
        executor.spin_until_future_complete(future, timeout_sec=2.0)
        executor.remove_node(self)

        if future.result() is not None:
            return future.result().current_state.label
        return "unknown"

    def _wait_for_unconfigured(self, node_name, timeout=15.0):
        # 노드가 unknown 상태일 때 unconfigured/inactive/active 가 될 때까지 대기
        start = time.time()
        while time.time() - start < timeout:
            state = self._get_state(node_name)
            print(f"[{node_name}] 상태 대기 중: {state}")
            if state in ('unconfigured', 'inactive', 'active'):
                return True
            time.sleep(1.0)
        print(f"[{node_name}] === 대기 시간 초과 (마지막 상태: {self._get_state(node_name)}) ===")
        return False

    def start_map_server(self, map_path, is_slam=False):
        print(f"=== 맵 변경 시작: {map_path} ===")

        all_nodes = ['/map_server', '/amcl'] + self.ordered_nodes

        for node in all_nodes:
            current = self._get_state(node)
            print(f"[{node}] 현재 상태: {current}")

            if current == 'active':
                continue

            # unknown 상태면 unconfigured 가 될 때까지 대기
            if current == 'unknown':
                print(f"[{node}] === unknown 상태 감지, unconfigured 대기 중 ===")
                if not self._wait_for_unconfigured(node):
                    print(f"[{node}] === 상태 전환 실패, 건너뜀 ===")
                    continue
                current = self._get_state(node)

            # unconfigured -> configure -> inactive
            if current == 'unconfigured':
                print(f"[{node}] Configuring...")
                ok = self._change_state(node, Transition.TRANSITION_CONFIGURE)
                if not ok:
                    print(f"[{node}] === configure 실패, 건너뜀 ===")
                    continue
                time.sleep(0.5)

            # inactive -> activate -> active
            print(f"[{node}] Activating...")
            self._change_state(node, Transition.TRANSITION_ACTIVATE)
            time.sleep(0.3)

            final = self._get_state(node)
            if final == 'active':
                print(f"[{node}] === 활성화 완료 ===")
            else:
                print(f"[{node}] === 활성화 후 상태: {final} ===")

        print("=== 모든 프로세스 완료 ===")
        return True

    def _publish_initialpose(self):
        pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.w = 1.0

        time.sleep(1)
        pub.publish(msg)
        print("=== Initial Pose 발행 완료 ===")