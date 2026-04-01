import requests
import json
import time
from rclpy.node import Node 
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
import sys
from database.node_service import NodeService
from database.map_service import map_service
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math
from std_msgs.msg import String, Int32

sys.path.append('/home/dev/fp_ws/src/fp_pkg')

PALLET_MAP = {
    0: {'carry_id': 0, 'color': 'BLUE'},
    1: {'carry_id': 1, 'color': 'RED'},
    2: {'carry_id': 2, 'color': 'YELLOW'},
}

class LLMController(Node):
    def __init__(self, model="qwen2.5:3b"):
        super().__init__('llm_controller')

        reliable_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        fork_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        
        self.url = "http://localhost:11434/api/generate"
        self.model = model
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.llm_goal_pub = self.create_publisher(PoseStamped, '/llm_goal', 10)
        self.fork_pub = self.create_publisher(String, '/fork_cmd', fork_qos)
        self.carry_id_pub   = self.create_publisher(Int32,    '/carry_id',   reliable_profile)
        self.get_unload_pub = self.create_publisher(String,   '/get_unload', reliable_profile)

        self.node_service = NodeService()

        # 로봇 위치 수신 토픽
        self.current_pose = None
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)

    def pose_callback(self, msg):
        self.current_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )


    def get_robot_command(self, user_text):
        try:
            prompt = f"""Convert Korean robot command to JSON only. No explanation.
                Output: {{"steps": [...]}}

                Types:
                - move: {{"type":"move","dir":"forward"|"backward","val":<meters>}}
                - rotate: {{"type":"rotate","dir":"left"|"right","angle":1.57}}
                - nav: {{"type":"nav","place":"<node_id>"}}
                - fork: {{"type":"fork","action":"UP"|"DOWN"}}
                - load_task: {{"type":"load_task","pallet":<0|1|2>}}
                - unload_task: {{"type":"unload_task","color":"BLUE"|"RED"|"YELLOW"}}

                Nodes:
                충전/충전소=CHRG001, 상차1=LOAD001, 상차2=LOAD002, 상차3=LOAD003
                경유1=NODE001, 경유2=NODE002, 경유3=NODE003, 경유4=NODE004
                하차1=UNLD001, 하차2=UNLD002, 하차3=UNLD003, 대기=WAIT001

                Pallet mapping:
                팔레트1/1번=0, 팔레트2/2번=1, 팔레트3/3번=2

                Rules:
                - 상차노드로 이동 후 상차 작업 → nav(LOAD001) + load_task
                - 하차노드로 이동 후 하차 작업 → nav(UNLD001) + unload_task
                - 상차/하차 작업 단독 명령 → load_task or unload_task only (nav 추가 금지)
                - 지게발/포크 단독 → fork only (nav 절대 추가 금지)
                - 복합명령("~한 후","~하고") → steps 순서대로

                Examples:
                "앞으로 1m 이동해" → {{"steps":[{{"type":"move","dir":"forward","val":1.0}}]}}
                "지게발 올려줘" → {{"steps":[{{"type":"fork","action":"UP"}}]}}
                "지게발 내려줘" → {{"steps":[{{"type":"fork","action":"DOWN"}}]}}
                "충전소 가줘" → {{"steps":[{{"type":"nav","place":"CHRG001"}}]}}
                "상차노드로 이동한 후 1번 팔레트 상하차 작업 수행해" → {{"steps":[{{"type":"nav","place":"LOAD001"}},{{"type":"load_task","pallet":0}}]}}
                "상차노드 이동 후 지게발 올려" → {{"steps":[{{"type":"nav","place":"LOAD001"}},{{"type":"fork","action":"UP"}}]}}

                Input: {user_text}
                Output:"""

            response = requests.post(self.url, json={
                "model": self.model,
                "prompt": prompt,
                "stream": False,
                "format": "json",
                "options": {
                    "temperature": 0.1,
                    "num_predict": 100,
                    "think": False
                }
            })
            result = self._validate_steps(user_text, json.loads(response.json()['response']))
            
            return result
        except Exception as e:
            print(f"LLM Error: {e}")
            return None

    def _validate_steps(self, user_text, result):
        if not result or 'steps' not in result:
            return result

        steps = result['steps']

        fork_keywords         = ['지게발', '포크', 'fork']
        nav_keywords          = ['이동', '가줘', '가', '노드', '장소', '위치', '으로']
        load_task_keywords    = ['상차 작업', '상차작업', '상차 수행']
        unload_task_keywords  = ['하차 작업', '하차작업', '하차 수행']

        # fork 단독 명령인데 nav가 붙은 경우 제거
        is_fork_only = (any(kw in user_text for kw in fork_keywords) and
                        not any(kw in user_text for kw in nav_keywords))
        if is_fork_only:
            original = steps[:]
            steps = [s for s in steps if s['type'] == 'fork']
            if steps != original:
                print(f"[검증] nav 강제 제거: {original} → {steps}")

        # 상차 작업 단독 명령인데 nav가 붙은 경우 제거
        is_load_only = (any(kw in user_text for kw in load_task_keywords) and
                        not any(kw in user_text for kw in nav_keywords))
        if is_load_only:
            steps = [s for s in steps if s['type'] == 'load_task']

        # 하차 작업 단독 명령인데 nav가 붙은 경우 제거
        is_unload_only = (any(kw in user_text for kw in unload_task_keywords) and
                          not any(kw in user_text for kw in nav_keywords))
        if is_unload_only:
            steps = [s for s in steps if s['type'] == 'unload_task']

        result['steps'] = steps
        print(f"[검증 완료] {user_text} → {result['steps']}")
        return result


    def get_coords_from_db(self, place_name: str):
        try:
            keyword_map = {
                '충전':  'CHRG001',
                '상차1': 'LOAD001',
                '상차2': 'LOAD002',
                '상차3': 'LOAD003',
                '경유1': 'NODE001',
                '경유2': 'NODE002',
                '경유3': 'NODE003',
                '경유4': 'NODE004',
                '하차1': 'UNLD001',
                '하차2': 'UNLD002',
                '하차3': 'UNLD003',
                '대기':  'WAIT001',
            }

            target_id = keyword_map.get(place_name, place_name.upper().strip())
            print(target_id)
            active_map = map_service.get_active_map()
            if not active_map:
                print("활성화된 맵이 없습니다.")
                return None

            nodes = self.node_service.get_nodes({'map_id': active_map['map_seq']})
            for node in nodes:
                if node['node_id'] == target_id:
                    return {
                        'x':   float(node['node_x_coord']),
                        'y':   float(node['node_y_coord']),
                        'yaw': 0.0
                    }

            print(f"'{place_name}' → '{target_id}' DB에서 찾지 못했습니다.")
            return None

        except Exception as e:
            print(f"DB 조회 오류: {e}")
            return None

    def _wait_until_arrived(self, target_x, target_y, tolerance=0.15, timeout=120):
        """
        목표 위치에 도달할 때까지 대기
        tolerance: 도착 판정 거리 (미터)
        timeout: 최대 대기 시간 (초)
        """
        start = time.time()
        while time.time() - start < timeout:
            if self.current_pose is not None:
                dx = self.current_pose[0] - target_x
                dy = self.current_pose[1] - target_y
                dist = math.sqrt(dx*dx + dy*dy)
                if dist < tolerance:
                    return True
            time.sleep(0.5)
        return False

    def publish_llm_goal(self, x, y, yaw=0.0):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        self.llm_goal_pub.publish(msg)

    def execute_robot_sequence(self, steps, socketio):
        print("================== execute_robot_sequence start ==================")
        msg = Twist()
        PUBLISH_RATE = 0.05

        for step in steps:

            if step['type'] == 'rotate':
                socketio.emit('chat_response', {'data': f"{step['dir']} 방향으로 회전 중..."})

                angular_speed = 0.3
                calibration   = 2.0
                duration      = (step['angle'] / angular_speed) * calibration
                msg.linear.x  = 0.0
                msg.angular.z = -angular_speed if step['dir'] == 'right' else angular_speed

                start = time.time()
                while time.time() - start < duration:
                    self.cmd_pub.publish(msg)
                    time.sleep(PUBLISH_RATE)

                msg.linear.x  = 0.0
                msg.angular.z = 0.0
                self.cmd_pub.publish(msg)
                time.sleep(0.5)

            elif step['type'] == 'move':
                direction = step['dir']
                socketio.emit('chat_response', {
                    'data': f"{step['val']}m {'전진' if direction == 'forward' else '후진'} 중..."
                })

                linear_speed  = 0.2
                duration      = step['val'] / linear_speed
                msg.angular.z = 0.0
                msg.linear.x  = linear_speed if direction == 'forward' else -linear_speed

                start = time.time()
                while time.time() - start < duration:
                    self.cmd_pub.publish(msg)
                    time.sleep(PUBLISH_RATE)

                msg.linear.x  = 0.0
                msg.angular.z = 0.0
                self.cmd_pub.publish(msg)
                time.sleep(0.5)

            elif step['type'] == 'nav':
                place_name = step['place']
                socketio.emit('chat_response', {'data': f"'{place_name}' 위치 검색 중..."})

                coords = self.get_coords_from_db(place_name)
                if coords is None:
                    socketio.emit('chat_response', {
                        'data': f"'{place_name}' 을(를) DB에서 찾을 수 없습니다."
                    })
                    continue

                socketio.emit('chat_response', {
                    'data': f"'{place_name}' 으로 이동 중... (x={coords['x']:.2f}, y={coords['y']:.2f})"
                })
                self.publish_llm_goal(coords['x'], coords['y'], coords['yaw'])

                # 다음 step이 있으면 도착 대기
                current_idx  = steps.index(step)
                has_next_step = current_idx < len(steps) - 1

                if has_next_step:
                    socketio.emit('chat_response', {'data': "목적지 도착 대기 중..."})
                    arrived = self._wait_until_arrived(
                        coords['x'], coords['y'],
                        tolerance=0.3,
                        timeout=60
                    )
                    if arrived:
                        socketio.emit('chat_response', {
                            'data': f"'{place_name}' 도착 완료! 다음 동작 수행합니다."
                        })
                    else:
                        socketio.emit('chat_response', {
                            'data': f"'{place_name}' 도착 시간 초과. 다음 동작을 수행합니다."
                        })

            elif step['type'] == 'fork':
                action = step['action']
                socketio.emit('chat_response', {
                    'data': f"지게발 {'올리는' if action == 'UP' else '내리는'} 중..."
                })

                fork_msg      = String()
                fork_msg.data = action
                self.fork_pub.publish(fork_msg)
                time.sleep(2.0)

                socketio.emit('chat_response', {
                    'data': f"지게발 {'올리기' if action == 'UP' else '내리기'} 완료"
                })

            elif step['type'] == 'load_task':
                pallet = step.get('pallet', 0)
                info   = PALLET_MAP.get(pallet, PALLET_MAP[0])

                socketio.emit('chat_response', {
                    'data': f"팔레트 {pallet+1}번 상차 작업 시작... (carry_id={info['carry_id']})"
                })

                carry_msg      = Int32()
                carry_msg.data = info['carry_id']
                self.carry_id_pub.publish(carry_msg)

                socketio.emit('chat_response', {'data': "상차 작업 명령 전송 완료"})

            elif step['type'] == 'unload_task':
                pallet = step.get('pallet', 0)
                info   = PALLET_MAP.get(pallet, PALLET_MAP[0])

                socketio.emit('chat_response', {
                    'data': f"팔레트 {pallet+1}번 하차 작업 시작... (색상={info['color']})"
                })

                unload_msg      = String()
                unload_msg.data = info['color']
                self.get_unload_pub.publish(unload_msg)

                socketio.emit('chat_response', {
                    'data': f"하차 작업 명령 전송 완료 ({info['color']})"
                })

        socketio.emit('chat_response', {'data': "명령 수행 완료 !"})
