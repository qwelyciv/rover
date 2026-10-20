import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
import serial
import json
import time
import math


class DriveMuxBridge(Node):
    """
    Main drive node for the rover.
    Handles switching between Nav2 (auto) and web UI (manual RC), converts Twist 
    messages into motor PWM + steering steps, and sends them to the ESP32 over UART.
    """

    def __init__(self):
        super().__init__('drive_mux_bridge')

        # Serial setup for ESP32
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)

        # Rover physical specs from CDR doc
        self.declare_parameter('wheelbase_m', 0.20)             # 200mm chassis length
        self.declare_parameter('wheel_radius_m', 0.07)          # 70mm wheels
        self.declare_parameter('motor_max_rpm', 251.0)          # FIT0186 12V geared motors
        self.declare_parameter('drivetrain_efficiency', 0.85)   # rough estimate for sand losses

        # Steering rack specs (16-tooth pinion gear + bipolar stepper)
        self.declare_parameter('pinion_teeth', 16)
        self.declare_parameter('gear_module_mm', 1.5)
        self.declare_parameter('steering_arm_length_mm', 45.0)
        self.declare_parameter('stepper_steps_per_rev', 200)
        self.declare_parameter('stepper_microsteps', 8)
        self.declare_parameter('max_steer_angle_deg', 35.0)     # max angle before wheel hits wishbone

        # Safety timeouts
        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.declare_parameter('mcu_hb_timeout_sec', 1.5)
        self.declare_parameter('rc_override_lockout_sec', 2.0)

        # Pull params into variables
        self.L = self.get_parameter('wheelbase_m').value
        self.r_wheel = self.get_parameter('wheel_radius_m').value
        self.max_rpm = self.get_parameter('motor_max_rpm').value
        self.efficiency = self.get_parameter('drivetrain_efficiency').value

        # Max speed math: v = (RPM * 2 * pi * r) / 60
        # Comes out to 1.84 m/s no-load (CDR target was 2 m/s, close enough)
        self.no_load_speed = (self.max_rpm * 2.0 * math.pi * self.r_wheel) / 60.0
        self.max_speed = self.no_load_speed * self.efficiency

        # Figure out how many stepper steps we need per meter of rack travel
        teeth = self.get_parameter('pinion_teeth').value
        module_m = self.get_parameter('gear_module_mm').value / 1000.0
        self.steer_arm_m = self.get_parameter('steering_arm_length_mm').value / 1000.0
        steps_per_rev = (self.get_parameter('stepper_steps_per_rev').value *
                         self.get_parameter('stepper_microsteps').value)

        pinion_circumference = math.pi * module_m * teeth
        self.steps_per_meter = steps_per_rev / pinion_circumference
        self.max_steer_rad = math.radians(self.get_parameter('max_steer_angle_deg').value)

        self.cmd_timeout = self.get_parameter('cmd_timeout_sec').value
        self.mcu_timeout = self.get_parameter('mcu_hb_timeout_sec').value
        self.rc_lockout = self.get_parameter('rc_override_lockout_sec').value

        # State tracking (using time.monotonic() so ntp time jumps don't mess up the watchdog)
        now = time.monotonic()
        self.state = "IDLE"          # IDLE, ACTIVE, TIMED_OUT, ESTOP, MCU_OFFLINE
        self.mode = "MANUAL"         # MANUAL or AUTO
        self.last_cmd_time = 0.0
        self.last_override_time = 0.0
        self.last_mcu_time = now
        self.seq = 0
        self.last_ack = -1

        self.get_logger().info(
            f"Drive node ready. Max speed: {self.max_speed:.2f} m/s (no-load: {self.no_load_speed:.2f} m/s)"
        )

        # Try opening serial port to ESP32
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self.esp_serial = None
        try:
            self.esp_serial = serial.Serial(port, baud, timeout=0.0)
            self.get_logger().info(f"Connected to ESP32 on {port}")
        except serial.SerialException as e:
            # Keep running even if unplugged so we can test with ros2 topic echo
            self.get_logger().warn(f"Couldn't open {port} ({e}). Running without hardware.")

        # Subscribers & publishers
        self.sub_nav = self.create_subscription(Twist, '/cmd_vel_nav', self.nav_callback, 10)
        self.sub_rc = self.create_subscription(Twist, '/cmd_vel_rc', self.rc_callback, 10)
        self.sub_mode = self.create_subscription(Bool, '/rover/auto_mode', self.mode_callback, 10)
        self.sub_kill = self.create_subscription(Bool, '/rover/kill_switch', self.kill_callback, 10)

        self.pub_status = self.create_publisher(String, '/rover/drive_status', 10)

        # Main 20Hz loop for watchdog and checking ESP32 responses
        self.create_timer(0.05, self.update_loop)

    def mode_callback(self, msg):
        """Toggle between AUTO (Nav2) and MANUAL (web UI)[cite: 1]."""
        if self.state == "ESTOP":
            self.get_logger().warn("Can't change mode while Kill Switch is active!")
            return

        now = time.monotonic()
        new_mode = "AUTO" if msg.data else "MANUAL"
        if new_mode == self.mode:
            return

        # Don't let it jump back to AUTO right after someone grabbed manual controls
        if new_mode == "AUTO" and (now - self.last_override_time) < self.rc_lockout:
            self.get_logger().warn("Ignoring AUTO switch - manual override happened too recently.")
            return

        self.mode = new_mode
        self.last_cmd_time = 0.0  # reset timer so old queued commands don't move the rover
        self.stop_rover("IDLE")
        self.get_logger().info(f"Switched drive mode to {self.mode}")

    def kill_callback(self, msg):
        """Software kill switch from the web dashboard[cite: 1]."""
        if msg.data:
            if self.state != "ESTOP":
                self.get_logger().error("KILL SWITCH PRESSED! Stopping motors.")
                self.stop_rover("ESTOP")
        else:
            if self.state == "ESTOP":
                self.get_logger().info("Kill switch reset. Back to MANUAL idle.")
                self.mode = "MANUAL"
                self.last_cmd_time = 0.0
                self.stop_rover("IDLE")

    def rc_callback(self, msg):
        """Manual WASD/Arrow keys from the web UI[cite: 1]. Always overrides Nav2[cite: 1]."""
        if self.state == "ESTOP":
            return

        now = time.monotonic()
        is_moving = abs(msg.linear.x) > 0.001 or abs(msg.angular.z) > 0.001

        # SW-03 requirement: manual input immediately kicks rover out of AUTO mode
        if self.mode == "AUTO" and is_moving:
            self.mode = "MANUAL"
            self.last_override_time = now
            self.get_logger().warn("Manual input detected! Overriding Nav2 -> MANUAL mode.")

        if self.mode == "MANUAL":
            self.last_cmd_time = now
            self.state = "ACTIVE"
            self.handle_twist(msg)

    def nav_callback(self, msg):
        """Velocity commands coming from Nav2 stack[cite: 1]."""
        if self.state == "ESTOP" or self.mode != "AUTO":
            return

        self.last_cmd_time = time.monotonic()
        self.state = "ACTIVE"
        self.handle_twist(msg)

    def handle_twist(self, msg):
        """
        Convert Twist (v, omega) into motor PWM and steering stepper steps.
        Since we use rack-and-pinion steering instead of diff drive, we need the 
        bicycle model: delta = atan(L * omega / v).
        """
        v = max(min(msg.linear.x, self.max_speed), -self.max_speed)
        omega = msg.angular.z

        # Calculate steering angle (delta)
        if abs(v) >= 0.05:
            delta = math.atan((self.L * omega) / abs(v))
        elif abs(omega) > 0.001:
            # If we're barely moving but trying to turn, just turn wheels to max angle
            delta = math.copysign(self.max_steer_rad, omega)
        else:
            delta = 0.0

        # Clamp to physical wishbone limits
        delta = max(min(delta, self.max_steer_rad), -self.max_steer_rad)

        # Map linear speed to 8-bit PWM (-255 to 255) for Cytron MDD10A drivers
        pwm = int(round((v / self.max_speed) * 255.0))
        pwm = max(min(pwm, 255), -255)

        # Convert steering angle into linear rack movement, then into stepper steps
        rack_shift_m = self.steer_arm_m * math.sin(delta)
        steer_steps = int(round(rack_shift_m * self.steps_per_meter))

        self.send_to_esp32(pwm, steer_steps, estop=False)

    def stop_rover(self, new_state):
        """Send a single stop packet when changing state so we don't spam UART."""
        self.state = new_state
        is_estop = (new_state == "ESTOP")
        self.send_to_esp32(0, 0, estop=is_estop)

    def send_to_esp32(self, pwm, steer_steps, estop=False):
        """
        Send JSON packet over UART[cite: 1].
        Added 'seq' counter so the ESP32 can run its own watchdog if the Pi crashes.
        """
        self.seq = (self.seq + 1) % 65536
        packet = {
            "seq": self.seq,
            "pwm": abs(pwm),
            "dir": 1 if pwm >= 0 else 0,
            "steer": steer_steps,
            "estop": 1 if estop else 0
        }
        line = json.dumps(packet) + '\n'

        if self.esp_serial and self.esp_serial.is_open:
            try:
                self.esp_serial.write(line.encode('utf-8'))
            except serial.SerialException as e:
                self.get_logger().error(f"Serial write error: {e}")
                self.state = "MCU_OFFLINE"

    def read_esp32_feedback(self, now):
        """Check if ESP32 sent back an ACK or heartbeat: {"ack": 12, "err": 0}"""
        if not self.esp_serial or not self.esp_serial.is_open:
            return

        try:
            while self.esp_serial.in_waiting > 0:
                raw = self.esp_serial.readline().decode('utf-8', errors='ignore').strip()
                if not raw:
                    continue
                data = json.loads(raw)
                self.last_mcu_time = now

                if "ack" in data:
                    self.last_ack = int(data["ack"])
                if data.get("err", 0) != 0:
                    self.get_logger().error(f"ESP32 fault code: {data['err']}")
                    self.stop_rover("ESTOP")
        except Exception:
            # Ignore garbled serial lines
            pass

    def update_loop(self):
        """Runs at 20Hz to check timeouts and publish rover status."""
        now = time.monotonic()
        self.read_esp32_feedback(now)

        # Watchdog: only trigger stop once when transitioning from ACTIVE to TIMED_OUT
        if self.state == "ACTIVE" and (now - self.last_cmd_time) > self.cmd_timeout:
            self.get_logger().warn("Watchdog timeout! No velocity commands received, stopping.")
            self.stop_rover("TIMED_OUT")

        # Check if ESP32 stopped talking to us
        if self.esp_serial and self.esp_serial.is_open:
            if (now - self.last_mcu_time) > self.mcu_timeout:
                if self.state not in ("MCU_OFFLINE", "ESTOP"):
                    self.get_logger().error("Lost heartbeat from ESP32!")
                    self.state = "MCU_OFFLINE"

        # Publish status for the web UI telemetry node
        status = {
            "state": self.state,
            "mode": self.mode,
            "seq": self.seq,
            "ack": self.last_ack
        }
        self.pub_status.publish(String(data=json.dumps(status)))


def main(args=None):
    rclpy.init(args=args)
    node = DriveMuxBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_rover("IDLE")
        if node.esp_serial and node.esp_serial.is_open:
            node.esp_serial.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()