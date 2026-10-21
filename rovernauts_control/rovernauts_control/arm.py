import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, Bool
import serial
import time
import threading


class ArmControllerNode(Node):
    """
    Controls the 4-DOF excavation arm via a Raspberry Pi Pico over USB serial.
    The arm uses 4x 28BYJ-48 steppers (modded to bipolar mode with step/dir drivers).
    Supports manual joint angles from the web UI and pre-programmed digging macros.
    """

    def __init__(self):
        super().__init__('arm_controller')

        # Serial port for the Pi Pico running MicroPython
        self.declare_parameter('pico_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)

        # Link lengths from CDR Table 1 (in cm, mostly here for reference/IK later)
        # Base: 8cm, Shoulder: 10cm, Elbow: 10cm, Scoop: 5cm
        self.link_lengths_cm = [8.0, 10.0, 10.0, 5.0]

        # Software angle limits (degrees) so the PETG links don't smash into the chassis
        # Order: [Base (360 deg), Shoulder, Elbow, Scoop]
        self.joint_limits = [
            (-180.0, 180.0),
            (-40.0, 90.0),
            (-90.0, 110.0),
            (-90.0, 90.0)
        ]

        # 28BYJ-48 gear ratio is ~64:1, 64 steps/rev -> ~4096 steps per 360 deg (~11.38 steps/deg)
        self.steps_per_deg = 4096.0 / 360.0

        self.current_angles = [0.0, 0.0, 0.0, 0.0]
        self.macro_active = False
        self.abort_macro = False

        # Connect to Pi Pico
        port = self.get_parameter('pico_port').value
        baud = self.get_parameter('baud_rate').value
        self.pico_serial = None
        try:
            self.pico_serial = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"Connected to Arm Pi Pico on {port}")
        except serial.SerialException as e:
            self.get_logger().warn(f"Pi Pico not found on {port} ({e}). Running in dummy mode.")

        # Subscribers for manual WASD control, macro buttons, and kill switch
        self.sub_joints = self.create_subscription(
            Float32MultiArray, '/arm/joint_targets', self.joint_callback, 10)
        self.sub_macro = self.create_subscription(
            String, '/arm/macro_cmd', self.macro_callback, 10)
        self.sub_kill = self.create_subscription(
            Bool, '/rover/kill_switch', self.kill_callback, 10)

        self.pub_status = self.create_publisher(String, '/arm/status', 10)

    def kill_callback(self, msg):
        """Stop any running digging sequence if the operator hits the red Kill Switch."""
        if msg.data:
            self.abort_macro = True
            if self.pico_serial and self.pico_serial.is_open:
                try:
                    self.pico_serial.write(b"STOP\n")
                except Exception:
                    pass
            self.pub_status.publish(String(data="ESTOP"))
            self.get_logger().error("Kill switch triggered! Halting arm steppers.")
        else:
            self.abort_macro = False
            self.pub_status.publish(String(data="IDLE"))

    def clamp_angles(self, angles):
        """Keep target angles inside safe mechanical bounds."""
        safe = []
        for i in range(4):
            low, high = self.joint_limits[i]
            val = max(min(float(angles[i]), high), low)
            safe.append(val)
        return safe

    def send_to_pico(self, angles, speed_deg_s=35.0):
        """Convert joint angles into stepper steps and send command line to Pico."""
        safe_angles = self.clamp_angles(angles)
        self.current_angles = safe_angles
        steps = [int(round(a * self.steps_per_deg)) for a in safe_angles]

        # Format expected by our MicroPython script on the Pico
        cmd_str = f"STEP:{steps[0]},{steps[1]},{steps[2]},{steps[3]}|SPD:{int(speed_deg_s)}\n"
        if self.pico_serial and self.pico_serial.is_open:
            try:
                self.pico_serial.write(cmd_str.encode('utf-8'))
            except serial.SerialException as e:
                self.get_logger().error(f"Lost serial connection to Pico: {e}")

    def joint_callback(self, msg):
        """Manual joint control from the web UI (ignored if a macro is currently running)."""
        if self.macro_active or self.abort_macro:
            return
        if len(msg.data) == 4:
            self.send_to_pico(msg.data)

    def macro_callback(self, msg):
        """Trigger pre-programmed sequences from the web UI buttons (Store, Deposit, Stow)."""
        cmd = msg.data.strip().upper()
        if self.macro_active or self.abort_macro:
            self.get_logger().warn("Arm is busy or stopped, ignoring macro request.")
            return

        if cmd in ("STORE", "DEPOSIT", "STOW"):
            # Run sequence in a thread so time.sleep() doesn't freeze the ROS2 node
            t = threading.Thread(target=self.run_macro_sequence, args=(cmd,), daemon=True)
            t.start()
        else:
            self.get_logger().warn(f"Unknown arm macro: {cmd}")

    def run_macro_sequence(self, name):
        self.macro_active = True
        self.pub_status.publish(String(data=f"MACRO_{name}"))
        self.get_logger().info(f"Starting arm macro: {name}")

        # Keyframes: ([base, shoulder, elbow, scoop], wait_time_sec)
        if name == "STORE":
            # Scoop ~100g of sand and dump it into the onboard container bay
            waypoints = [
                ([0.0, 40.0, -25.0, -45.0], 2.0),    # reach out in front of rover
                ([0.0, -35.0, -60.0, -45.0], 2.0),   # drop scoop into the sand
                ([0.0, -20.0, -40.0, 65.0], 1.5),    # curl bucket to scoop sand
                ([0.0, 70.0, 45.0, 65.0], 2.5),      # lift up (keep CoG low)
                ([180.0, 70.0, 45.0, 65.0], 3.0),    # spin base 180 deg over the storage bay
                ([180.0, 50.0, 80.0, -80.0], 2.0),   # tilt scoop to dump sand into box
                ([0.0, 85.0, -85.0, 0.0], 2.5)       # fold back to safe driving position
            ]
        elif name == "DEPOSIT":
            # Dump collected sample at the return zone
            waypoints = [
                ([180.0, 55.0, 75.0, -90.0], 2.0),
                ([0.0, 85.0, -85.0, 0.0], 2.0)
            ]
        elif name == "STOW":
            # Tuck arm in tight so we fit inside the 0.03 m^3 box & survive Airbus vibration test
            waypoints = [
                ([0.0, 90.0, -90.0, 90.0], 3.0)
            ]

        for angles, wait_s in waypoints:
            if self.abort_macro:
                self.get_logger().warn(f"Macro {name} aborted!")
                break
            self.send_to_pico(angles)
            time.sleep(wait_s)

        if not self.abort_macro:
            self.pub_status.publish(String(data="IDLE"))
            self.get_logger().info(f"Finished arm macro: {name}")
        self.macro_active = False


def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.pico_serial and node.pico_serial.is_open:
            node.pico_serial.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

