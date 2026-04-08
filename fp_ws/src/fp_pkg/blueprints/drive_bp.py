from flask import Blueprint, request, jsonify
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

driving_node = None

_socketio = None  # ★ 추가


def set_socketio(sio):  # ★ 추가
    global _socketio
    _socketio = sio

class DrivingNode(Node):
    def __init__(self):
        super().__init__('driving_node')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        self.carry_publisher             = self.create_publisher(Int32,       '/carry_id',               qos_profile)
        self.waypoints_publisher         = self.create_publisher(PoseStamped, '/waypoint_drive',          qos_profile)
        self.return_home_publisher       = self.create_publisher(Bool,        '/return_home',             qos_profile)
        self.emergency_stop_web_publisher    = self.create_publisher(Bool,    '/emergency_stop_web',      qos_profile)
        self.emergency_resolve_web_publisher = self.create_publisher(Bool,    '/emergency_resolve_web',   qos_profile)

        # ★ 소켓 이벤트용 구독
        # 아두이노발 비상정지/해제 구독 → 웹 모달 제어
        self.sub_emergency_arduino = self.create_subscription(Bool, '/emergency_stop_arduino',    self._on_emergency_arduino,         10)
        self.sub_resolve_arduino   = self.create_subscription(Bool, '/emergency_resolve_arduino', self._on_emergency_resolve_arduino, 10)
        # 웹발 비상정지 구독 → 웹 모달 제어 (자기 에코지만 모달 오픈용)
        self.sub_emergency_web     = self.create_subscription(Bool, '/emergency_stop_web',        self._on_emergency_web,             qos_profile)
        self.sub_resolve_web = self.create_subscription(Bool, '/emergency_resolve_web', self._on_emergency_resolve_web, qos_profile)
        self.sub_emergency_resolved = self.create_subscription(Bool, '/emergency_resolved', self._on_emergency_resolved, qos_profile)


    def _on_emergency_resolved(self, msg):
        """astar가 실제로 해제 완료했을 때만 소켓 전송"""
        if msg.data and _socketio:
            _socketio.emit('emergency_resolved', {})
            self.get_logger().info("[DrivingNode] 비상정지 해제 완료 소켓 전송")

    def _on_emergency_resolve_web(self, msg):
        if msg.data and _socketio:
            _socketio.emit('emergency_resolved', {'source': 'web'})
            self.get_logger().info("[DrivingNode] 웹 해제 소켓 전송")
            
    def _on_emergency_arduino(self, msg):
        """아두이노발 비상정지 → 웹 모달 오픈 + 해제버튼 숨김"""
        if msg.data and _socketio:
            _socketio.emit('emergency_triggered', {'data': True, 'source': 'arduino'})
            self.get_logger().info("[DrivingNode] 아두이노 비상정지 소켓 전송")

    def _on_emergency_resolve_arduino(self, msg):
        """아두이노 버튼 해제 → 웹 모달 닫기"""
        if msg.data:
            self.get_logger().info("[DrivingNode] 아두이노 해제 소켓 전송")

    def _on_emergency_web(self, msg):
        """웹발 비상정지 → 웹 모달 오픈 + 해제버튼 표시"""
        if msg.data and _socketio:
            _socketio.emit('emergency_triggered', {'data': True, 'source': 'web'})
            self.get_logger().info("[DrivingNode] 웹 비상정지 소켓 전송")


def get_driving_node():
    global driving_node
    if driving_node is None:
        # 이때 딱 한 번만 노드를 생성
        driving_node = DrivingNode()
    return driving_node

drive_bp = Blueprint('drive_bp', __name__)

@drive_bp.route('/startDrive.do', methods=['POST'])
def start_drive():
    print(f"======================= start_drive start =======================")
    node = get_driving_node()
    data = request.get_json()
    try:
        msg = Int32()
        msg.data = data['palletId']
        driving_node.carry_publisher.publish(msg)
        print(f"======================= start_drive {msg} =======================")
        return jsonify({"result": "success", "message": f"{int(data['palletId']) + 1}번 상/하차 시작"})
    except Exception as e:
        return jsonify({"result": "error", "message": str(e)})

@drive_bp.route('/waypointDrive.do', methods=['POST'])
def waypoint_drive():
    print(f"======================= waypoint_drive start =======================")
    node = get_driving_node()
    data = request.get_json()

    # data["x"], data["y"], data["yaw"]
    print(data)
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.pose.position.x = float(data['x'])
    msg.pose.position.y = float(data['y'])

    # yaw → quaternion 변환
    msg.pose.orientation.z = float(data['qz'])
    msg.pose.orientation.w = float(data['qw'])
    
    driving_node.waypoints_publisher.publish(msg)
    try:
       
        print(f"======================= waypoint_drive {msg} =======================")
        return jsonify({"result": "success", "message": f" 주행 시작"})
    except Exception as e:
        return jsonify({"result": "error", "message": str(e)})

# 중단 및 복귀 버튼 기능
@drive_bp.route('/abortReturn.do', methods=['POST'])
def abort_return():
    print(f"======================= abort_return start =======================")
    node = get_driving_node()
    try:
        msg = Bool()
        msg.data = True
        node.return_home_publisher.publish(msg)
        print(f"======================= abort_return published =======================")
        return jsonify({"result": "success", "message": "중단 및 복귀 명령 전송 완료"})
    except Exception as e:
        return jsonify({"result": "error", "message": str(e)})

# 비상정지 버튼 기능
@drive_bp.route('/emergencyStop.do', methods=['POST'])
def emergency_stop():
    print(f"======================= emergency_stop start =======================")
    node = get_driving_node()
    try:
        msg = Bool()
        msg.data = True
        node.emergency_stop_web_publisher.publish(msg)
        print(f"======================= emergency_stop published =======================")
        return jsonify({"result": "success", "message": "비상 정지 명령 전송 완료"})
    except Exception as e:
        return jsonify({"result": "error", "message": str(e)})

@drive_bp.route('/emergencyResolve.do', methods=['POST'])
def emergency_resolve():
    node = get_driving_node()
    try:
        msg = Bool()
        msg.data = True
        node.emergency_resolve_web_publisher.publish(msg)
        return jsonify({"result": "success", "message": "비상정지 해제 완료"})
    except Exception as e:
        return jsonify({"result": "error", "message": str(e)})