import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String, Float32MultiArray
import asyncio
import websockets
import json
import psutil
import threading
import math


class GCSTelemetryNode(Node):
    """
    WebSocket bridge between the ROS2 system on the Pi 4 and our web UI (GCS).
    Sends 1Hz system stats (CPU, temp, Wi-Fi signal) + IMU angles to the browser,
    and forwards operator keyboard/button inputs back into ROS2 topics.
    """

    def __init__(self):
        super().__init__('gcs_telemetry_node')

        # WebSocket port and external TP-Link Wi-Fi adapter interface
        self.declare_parameter('ws_port', 8765)
        self.declare_parameter('wifi_interface', 'wlan0')

        self.ws_port = self.get_parameter('ws_port').value
        self.wifi_iface = self.get_parameter('wifi_interface').value

        # Publishers for commands coming from the web dashboard
        self.pub_rc_vel = self.create_publisher(Twist, '/cmd_vel_rc', 10)
        self.pub_mode = self.create_publisher(Bool, '/rover/auto_mode', 10)
        self.pub_kill = self.create_publisher(Bool, '/rover/kill_switch', 10)
        self.pub_arm_macro = self.create_publisher(String, '/arm/macro_cmd', 10)
        self.pub_arm_joints = self.create_publisher(Float32MultiArray, '/arm/joint_targets', 10)

        # Subscribers for sensor data and subsystem states
        self.sub_imu = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.sub_drive = self.create_subscription(String, '/rover/drive_status', self.drive_status_callback, 10)
        self.sub_arm = self.create_subscription(String, '/arm/status', self.arm_status_callback, 10)

        # Latest rover state to include in the telemetry packet
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.drive_status = {"state": "IDLE", "mode": "MANUAL"}
        self.arm_status = "IDLE"

        self.clients = set()
        self.loop = None

        # SW-02 requirement: refresh telemetry at 1Hz minimum
        self.create_timer(1.0, self.broadcast_telemetry)

        # Run asyncio WebSocket server in a background thread so it doesn't block rclpy.spin()
        self.ws_thread = threading.Thread(target=self.run_ws_server, daemon=True)
        self.ws_thread.start()
        self.get_logger().info(f"GCS WebSocket server started on port {self.ws_port}")

    def drive_status_callback(self, msg):
        """Keep track of current drive mode (MANUAL/AUTO) and ESP32 status."""
        try:
            self.drive_status = json.loads(msg.data)
        except Exception:
            pass

    def arm_status_callback(self, msg):
        """Keep track of whether the 4-DOF arm is idle or running a macro."""
        self.arm_status = msg.data

    def imu_callback(self, msg):
        """
        Convert quaternion from ICM-20948 IMU into roll/pitch/yaw (degrees).
        We need roll and pitch on the UI to make sure we don't tip over on >30 deg slopes.
        """
        q = msg.orientation

        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

        # Pitch (y-axis rotation)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(min(sinp, 1.0), -1.0)  # clamp so asin doesn't throw domain error
        self.pitch = math.degrees(math.asin(sinp))

        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    def get_pi_temp(self):
        """Read Raspberry Pi 4 CPU temperature from Linux thermal zone."""
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                return round(float(f.read().strip()) / 1000.0, 1)
        except Exception:
            # Fallback value when testing on a laptop/VM
            return 42.5

    def get_wifi_rssi(self):
        """
        Read Wi-Fi signal strength (dBm) from /proc/net/wireless for the TL-WN722N adapter.
        From CDR link budget calc: expected ~ -47 dBm at 30m, receiver sensitivity limit is -68 dBm.
        """
        try:
            with open("/proc/net/wireless", "r") as f:
                for line in f:
                    if self.wifi_iface in line:
                        parts = line.split()
                        # parts[3] is link level (e.g. "-48."), strip trailing dot before casting
                        return int(float(parts[3].rstrip('.')))
        except Exception:
            pass
        return -48  # default random value if interface isn't up

    def broadcast_telemetry(self):
        """Build JSON packet and push it to all connected browsers at 1Hz."""
        if not self.clients or self.loop is None:
            return

        # Trigger warning on UI if incline gets close to the 30 deg stability limit
        tilt_warning = abs(self.roll) > 25.0 or abs(self.pitch) > 25.0

        packet = {
            "type": "TELEMETRY",
            "cpu_percent": psutil.cpu_percent(),
            "temp_c": self.get_pi_temp(),
            "rssi_dbm": self.get_wifi_rssi(),
            "drive": self.drive_status,
            "arm_state": self.arm_status,
            "imu": {
                "roll": round(self.roll, 1),
                "pitch": round(self.pitch, 1),
                "yaw": round(self.yaw, 1),
                "tilt_warning": tilt_warning
            }
        }

        payload = json.dumps(packet)
        asyncio.run_coroutine_threadsafe(self.send_to_clients(payload), self.loop)

    async def send_to_clients(self, message):
        """Send telemetry string to all active WebSocket connections."""
        for ws in list(self.clients):
            try:
                await ws.send(message)
            except websockets.exceptions.ConnectionClosed:
                self.clients.discard(ws)

    async def ws_handler(self, websocket):
        """Handle incoming WebSocket connections from the operator's browser."""
        self.clients.add(websocket)
        self.get_logger().info("Web UI client connected.")
        try:
            async for message in websocket:
                self.parse_web_command(message)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.clients.discard(websocket)
            self.get_logger().info("Web UI client disconnected.")

    def parse_web_command(self, raw_msg):
        """
        Parse JSON commands from the web interface:
        - Arrow keys -> rover movement (/cmd_vel_rc)
        - WASD keys  -> manual arm joint angles (/arm/joint_targets)
        - Buttons    -> Auto/RC toggle, Store, Deposit, Stow, Kill Switch
        """
        try:
            data = json.loads(raw_msg)
            cmd = data.get("cmd")

            if cmd == "MOVE":
                twist = Twist()
                twist.linear.x = float(data.get("linear", 0.0))
                twist.angular.z = float(data.get("angular", 0.0))
                self.pub_rc_vel.publish(twist)

            elif cmd == "ARM_MANUAL":
                # Expects a list of 4 angles: [base, shoulder, elbow, scoop]
                angles = data.get("angles", [0.0, 0.0, 0.0, 0.0])
                if len(angles) == 4:
                    msg = Float32MultiArray()
                    msg.data = [float(a) for a in angles]
                    self.pub_arm_joints.publish(msg)

            elif cmd == "MODE_TOGGLE":
                is_auto = bool(data.get("auto", False))
                self.pub_mode.publish(Bool(data=is_auto))

            elif cmd == "MACRO":
                # e.g. "STORE", "DEPOSIT", or "STOW"
                action = str(data.get("action", "STORE")).upper()
                self.pub_arm_macro.publish(String(data=action))

            elif cmd == "KILL_SWITCH":
                engaged = bool(data.get("engaged", True))
                self.pub_kill.publish(Bool(data=engaged))

        except Exception as e:
            self.get_logger().warn(f"Bad JSON command from web UI: {e}")

    def run_ws_server(self):
        """Setup asyncio event loop for the WebSocket server thread."""
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)

        async def server_main():
            async with websockets.serve(self.ws_handler, "0.0.0.0", self.ws_port):
                await asyncio.Future()  # run forever

        self.loop.run_until_complete(server_main())


def main(args=None):
    rclpy.init(args=args)
    node = GCSTelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()