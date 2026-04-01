import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        try:
            import serial
            self.ser = serial.Serial('/dev/ttyACM1', 9600, timeout=0.01)
            self.ser.reset_input_buffer()
            self.get_logger().info("✅ Arduino 시리얼 연결 성공")
        except Exception as e:
            self.ser = None
            self.get_logger().error(f"❌ Arduino 시리얼 연결 실패: {e}")

        # ── 퍼블리셔 ──
        self.pub_emergency_arduino   = self.create_publisher(Bool, '/emergency_stop_arduino',    10)
        self.pub_emergency_resolve_arduino = self.create_publisher(Bool, '/emergency_resolve_arduino', 10)
        self.pub_forklift_done       = self.create_publisher(Bool, '/forklift_done',             10)

        # ── 서브스크라이버 ──
        self.sub_forklift_cmd  = self.create_subscription(String, '/forklift_cmd',         self.forklift_cmd_callback,    10)
        self.sub_mission_start = self.create_subscription(Bool,   '/mission_start',        self.mission_start_callback,   10)
        self.sub_mission_done  = self.create_subscription(Bool,   '/mission_done',         self.mission_done_callback,    10)

        # ★ 웹발 비상정지/해제만 구독 (아두이노발은 구독 안 함 — 에코 방지)
        self.sub_emergency_web         = self.create_subscription(Bool, '/emergency_stop_web',     self.emergency_stop_web_callback,    10)
        self.sub_emergency_resolve_web = self.create_subscription(Bool, '/emergency_resolve_web',  self.emergency_resolve_web_callback, 10)

        self.timer = self.create_timer(0.05, self.serial_read_loop)
        self.get_logger().info("🤖 Arduino Bridge 시작!")

    def forklift_cmd_callback(self, msg):
        cmd = msg.data
        if cmd == 'LOAD':
            self.serial_write(b'W\n')
            self.get_logger().info("🔼 상차 명령 → 아두이노 'W' 전송")
        elif cmd == 'UNLD':
            self.serial_write(b'W\n')
            self.get_logger().info("🔽 하차 명령 → 아두이노 'W' 전송")
        elif cmd == 'HOME':
            self.serial_write(b'F\n')
            self.get_logger().info("🏠 귀환 완료 → 아두이노 'F' 전송")
        elif cmd == 'STOP':
            self.serial_write(b'E\n')
            self.get_logger().warn("🚨 SAFE_EXIT STOP → 아두이노 'E' 전송")

    def mission_start_callback(self, msg):
        if msg.data:
            self.serial_write(b'D\n')
            self.get_logger().info("🟢 미션 시작 → 아두이노 'D' 전송")

    def mission_done_callback(self, msg):
        if msg.data:
            self.serial_write(b'F\n')
            self.get_logger().info("🏁 미션 완료 → 아두이노 'F' 전송")

    def emergency_stop_web_callback(self, msg):
        """웹발 비상정지 → 아두이노 'E' 전송"""
        if msg.data:
            self.serial_write(b'E\n')
            self.get_logger().warn("🚨 웹 비상정지 → 아두이노 'E' 전송")

    def emergency_resolve_web_callback(self, msg):
        """웹발 해제 → 아두이노 'R' 전송"""
        if msg.data:
            self.serial_write(b'R\n')
            self.get_logger().info("✅ 웹 비상정지 해제 → 아두이노 'R' 전송")

    def serial_read_loop(self):
        if self.ser is None or self.ser.in_waiting == 0:
            return
        try:
            raw_data = self.ser.readline()
            line = raw_data.decode('utf-8', errors='ignore').strip()
            if not line:
                return

            self.get_logger().info(f"아두이노 수신: '{line}'")

            if line == 'E':
                # ★ 아두이노발 비상정지 → /emergency_stop_arduino 발행
                msg = Bool(); msg.data = True
                self.pub_emergency_arduino.publish(msg)
                self.get_logger().warn("⚠️  아두이노 비상정지 → /emergency_stop_arduino 퍼블리시")

            elif line == 'R':
                # ★ 아두이노 버튼 해제 → /emergency_resolve_arduino 발행
                msg = Bool(); msg.data = True
                self.pub_emergency_resolve_arduino.publish(msg)
                self.get_logger().info("✅ 아두이노 해제 버튼 → /emergency_resolve_arduino 퍼블리시")

            elif line == 'FIN':
                done_msg = Bool(); done_msg.data = True
                self.pub_forklift_done.publish(done_msg)
                self.get_logger().info("✅ 아두이노 작업 완료 → /forklift_done 퍼블리시")

            elif line.startswith('Mode:'):
                self.get_logger().info(f"아두이노 상태: {line}")

        except Exception as e:
            self.get_logger().error(f"Serial Read Error: {e}")

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