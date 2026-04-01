import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import serial


class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        # ── 아두이노 시리얼 연결 ──
        try:
            self.ser = serial.Serial('/dev/ttyACM1', 9600, timeout=0.01)
            self.ser.reset_input_buffer()  # ★ 아두이노 부팅 쓰레기 바이트 제거
            self.get_logger().info("✅ Arduino 시리얼 연결 성공")
        except Exception as e:
            self.ser = None
            self.get_logger().error(f"❌ Arduino 시리얼 연결 실패: {e}")

        # ── 퍼블리셔 (파이 → 리모트PC) ──
        self.pub_emergency     = self.create_publisher(Bool, '/emergency_stop', 10)
        self.pub_forklift_done = self.create_publisher(Bool, '/forklift_done',  10)

        # ── 서브스크라이버 (리모트PC → 파이) ──
        self.sub_forklift_cmd  = self.create_subscription(
            String, '/forklift_cmd',  self.forklift_cmd_callback,  10)
        self.sub_mission_start = self.create_subscription(
            Bool,   '/mission_start', self.mission_start_callback, 10)
        self.sub_mission_done  = self.create_subscription(
            Bool,   '/mission_done',  self.mission_done_callback,  10)

        # 웹 비상정지 토픽 (나중에 웹 연동 시 사용)
        self.sub_web_emergency = self.create_subscription(
            Bool, '/web_emergency', self.web_emergency_callback, 10)

        self.sub_emergency_resolve = self.create_subscription(
            Bool, '/emergency_resolve', self.emergency_resolve_callback, 10)

        # 아두이노 시리얼 수신 타이머
        self.timer = self.create_timer(0.05, self.serial_read_loop)

        self.get_logger().info("🤖 Arduino Bridge 시작!")

    # ─────────────────────────────────────────────
    # 리모트PC → 파이 → 아두이노
    # ─────────────────────────────────────────────
    def forklift_cmd_callback(self, msg):
        cmd = msg.data
        if cmd == 'LOAD':
            self.serial_write(b'W\n')
            self.get_logger().info("🔼 상차 명령 → 아두이노 'W' 전송")
            # ─────────────────────────────────────
            # TODO: 상차 지게차 시리얼 명령 추가 예정
            pass
            # ─────────────────────────────────────

        elif cmd == 'UNLD':
            self.serial_write(b'W\n')
            self.get_logger().info("🔽 하차 명령 → 아두이노 'W' 전송")
            # ─────────────────────────────────────
            # TODO: 하차 지게차 시리얼 명령 추가 예정
            pass
            # ─────────────────────────────────────

    def mission_start_callback(self, msg):
        """미션 시작 → 아두이노 'D' (주행 모드: 초록 LED)"""
        if msg.data:
            self.serial_write(b'D\n')
            self.get_logger().info("🟢 미션 시작 → 아두이노 'D' 전송")

    def mission_done_callback(self, msg):
        """미션 완료 → 아두이노 'F' (작업 종료)"""
        if msg.data:
            self.serial_write(b'F\n')
            self.get_logger().info("🏁 미션 완료 → 아두이노 'F' 전송")

    def web_emergency_callback(self, msg):
        """웹 비상정지/해제 (나중에 웹 연동 시 사용)"""
        if msg.data:
            self.serial_write(b'E\n')
            emergency_msg = Bool()
            emergency_msg.data = True
            self.pub_emergency.publish(emergency_msg)
            self.get_logger().warn("⚠️  웹 비상정지 → 아두이노 'E' + /emergency_stop 퍼블리시")
        else:
            self.serial_write(b'R\n')
            emergency_msg = Bool()
            emergency_msg.data = False
            self.pub_emergency.publish(emergency_msg)
            self.get_logger().info("✅ 웹 비상정지 해제 → 아두이노 'R' + /emergency_stop(False) 퍼블리시")
    def emergency_resolve_callback(self, msg):
        """리모트PC O키 → 아두이노 'R' (비상정지 해제)"""
        if msg.data:
            self.serial_write(b'R\n')
            self.get_logger().info("✅ 비상정지 해제 → 아두이노 'R' 전송")

    # ─────────────────────────────────────────────
    # 아두이노 → 파이 → 리모트PC
    # ─────────────────────────────────────────────
    def serial_read_loop(self):
        if self.ser is None or self.ser.in_waiting == 0:
            return
        try:
            raw_data = self.ser.readline()

            # ★ 쓰레기 바이트 무시
            line = raw_data.decode('utf-8', errors='ignore').strip()

            if not line:
                return

            self.get_logger().info(f"아두이노 수신: '{line}'")  # 디버그용

            # ★ 정확히 'E' 한 글자일 때만 비상정지 처리
            if line == 'E':
                msg = Bool()
                msg.data = True
                self.pub_emergency.publish(msg)
                self.get_logger().warn("⚠️  아두이노 비상정지 감지 → /emergency_stop 퍼블리시")

            # 지게차 작업 완료 신호 (아두이노에서 'FIN' 전송 시)
            elif line == 'FIN':
                done_msg = Bool()
                done_msg.data = True
                self.pub_forklift_done.publish(done_msg)
                self.get_logger().info("✅ 아두이노 작업 완료 신호 수신 → /forklift_done 퍼블리시")

            # Mode: Drive, Mode: Work 같은 일반 응답은 로그만
            elif line.startswith('Mode:'):
                self.get_logger().info(f"아두이노 상태 응답: {line}")

        except Exception as e:
            self.get_logger().error(f"Serial Read Error: {e}")

    # ─────────────────────────────────────────────
    # 헬퍼
    # ─────────────────────────────────────────────
    def serial_write(self, data: bytes):
        if self.ser:
            try:
                self.ser.write(data)
            except Exception as e:
                self.get_logger().error(f"Serial Write Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
