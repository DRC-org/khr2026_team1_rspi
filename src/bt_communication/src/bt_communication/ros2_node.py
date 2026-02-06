import asyncio
import json
import threading
from typing import Optional

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.action import ActionClient
# Ensure nav2_msgs is available in the environment
try:
    from nav2_msgs.action import NavigateToPose
except ImportError:
    NavigateToPose = None

from .gatt_server import BluetoothGATTServer


class BluetoothROS2Node(Node):
    def __init__(self):
        super().__init__("bluetooth_node")

        # Declare parameters
        self.declare_parameter("hci_transport", "usb:0")

        # Get parameters
        self.hci_transport: str = self.get_parameter("hci_transport").value or "usb:0"

        # Publisher: Send commands to robot (cmd_vel)
        self.cmd_vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)

        # Subscriber: Receive robot status (odom)
        self.odom_subscriber = self.create_subscription(
            Odometry, "odom", self.on_odom_received, 10
        )
        
        # Action Client for Nav2
        if NavigateToPose:
            self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        else:
            self._action_client = None
            self.get_logger().warning("nav2_msgs not found. Navigation actions disabled.")

        # Bluetooth server
        self.ble_server: Optional[BluetoothGATTServer] = None
        self.ble_thread: Optional[threading.Thread] = None
        self.event_loop: Optional[asyncio.AbstractEventLoop] = None

        self.get_logger().info("Bluetooth ROS2 Node initialized")
        self.get_logger().info(f"HCI Transport: {self.hci_transport}")

    def start_bluetooth_server(self):
        """Start Bluetooth server in a separate thread"""
        self.get_logger().info("Starting Bluetooth server thread...")

        def run_ble_server():
            """Run Bluetooth server in background thread"""
            self.event_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.event_loop)

            try:
                self.ble_server = BluetoothGATTServer(
                    on_data_received=self.on_bluetooth_data_received
                )
                self.event_loop.run_until_complete(
                    self.ble_server.start(self.hci_transport)
                )
            except KeyboardInterrupt:
                self.get_logger().info("Bluetooth server interrupted")
            except Exception as e:
                self.get_logger().error(f"Bluetooth server error: {e}")
            finally:
                self.event_loop.close()

        self.ble_thread = threading.Thread(target=run_ble_server, daemon=True)
        self.ble_thread.start()
        self.get_logger().info("Bluetooth server thread started")

    def on_bluetooth_data_received(self, data_str: str):
        """Handle data received from Bluetooth client (JSON) -> Publish to cmd_vel or Action"""
        try:
            # Expected JSON: {"type": "joystick", "l_x": ..., "l_y": ..., "r": ...}
            # or {"type": "navigate", "x": ..., "y": ...}
            data = json.loads(data_str)
            msg_type = data.get("type")
            
            if msg_type == "joystick":
                twist = Twist()
                # Mapping joystick (screen coords) to robot velocity
                # Joystick: Up (-y), Right (+x)
                # Robot:    Forward (+x), Left (+y)
                
                # Coefficients (Assuming joystick range approx -70 to 70)
                JOY_SCALE = 70.0
                MAX_LIN_VEL = 1.0  # m/s
                MAX_ANG_VEL = 2.0  # rad/s

                l_x = float(data.get("l_x", 0.0))
                l_y = float(data.get("l_y", 0.0))
                r   = float(data.get("r", 0.0))

                # Logic:
                # Forward (Robot +x) <= Joystick Up (-l_y)
                # Left (Robot +y)    <= Joystick Left (-l_x)
                # Left Turn (+ang)   <= Joystick Left (-r) ?? Usually Right Stick X controls turn.
                # If R stick right -> turns right (negative ang z)
                
                twist.linear.x = -(l_y / JOY_SCALE) * MAX_LIN_VEL
                twist.linear.y = -(l_x / JOY_SCALE) * MAX_LIN_VEL
                twist.angular.z = -(r / JOY_SCALE) * MAX_ANG_VEL
                
                # Simple clamping if valid range is exceeded
                # ...

                self.cmd_vel_publisher.publish(twist)
                
            elif msg_type == "navigate":
                if self._action_client:
                    # Field coordinates from App (mm) -> ROS (m)
                    target_x_mm = float(data.get("x", 0.0))
                    target_y_mm = float(data.get("y", 0.0))
                    
                    self.send_navigation_goal(target_x_mm / 1000.0, target_y_mm / 1000.0)
                else:
                    self.get_logger().warning("Navigate action requested but client not available.")

        except json.JSONDecodeError:
            self.get_logger().warning(f"Invalid JSON received: {data_str}")
        except Exception as e:
            self.get_logger().error(f"Error processing bluetooth data: {e}")

    def send_navigation_goal(self, x_m, y_m):
        """Send NavigateToPose action goal"""
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warning("NavigateToPose action server not available")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = x_m
        goal_msg.pose.pose.position.y = y_m
        goal_msg.pose.pose.orientation.w = 1.0 # Facing forward (or use current theta?)

        self.get_logger().info(f"Sending navigation goal: x={x_m}, y={y_m}")
        
        self._action_client.send_goal_async(goal_msg)

    def on_odom_received(self, msg: Odometry):
        """Handle odometry data -> Send to Bluetooth client"""
        if self.ble_server and self.event_loop:
            try:
                # Create simple JSON for visualization
                # Note: sending minimal data to keep bandwidth low
                response = {
                    "type": "robot_pos",
                    "x": round(msg.pose.pose.position.x * 1000, 1), # Convert to mm if App expects large numbers?
                    # App uses: bottom: calc(${robotPosY} / 7000 * 80svh)
                    # If robotPosY is 3500 (3.5m), it should be correct. 
                    # msg.pose.position.x is in meters. 3.5m -> 3.5.
                    # Wait, App State init: useState(388). 
                    # If App expects mm, then we must send mm.
                    # Let's assume App expects mm based on "7000" divisor.
                    
                    "y": round(msg.pose.pose.position.y * 1000, 1),
                    "angle": 0.0 # TODO: Convert quaternion to Euler deg
                }
                
                # Calculate yaw from quaternion
                q = msg.pose.pose.orientation
                siny_cosp = 2 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
                yaw_rad = math.atan2(siny_cosp, cosy_cosp)
                response["angle"] = round(math.degrees(yaw_rad), 1)

                
                # Send asynchronously
                json_str = json.dumps(response)
                asyncio.run_coroutine_threadsafe(
                    self.ble_server.send_data(json_str), self.event_loop
                )
            except Exception as e:
                self.get_logger().warning(f"Failed to send odom to BLE: {e}")

    def destroy_node(self):
        self.get_logger().info("Shutting down Bluetooth node")
        super().destroy_node()

