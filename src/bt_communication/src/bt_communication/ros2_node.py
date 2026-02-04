import asyncio
import json
import threading
from typing import Optional

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

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
        """Handle data received from Bluetooth client (JSON) -> Publish to cmd_vel"""
        try:
            # Expected JSON: {"vx": 0.5, "vy": 0.0, "omega": 0.1}
            data = json.loads(data_str)
            
            twist = Twist()
            # Safety checks and get values with defaults
            twist.linear.x = float(data.get("vx", 0.0))
            twist.linear.y = float(data.get("vy", 0.0))
            twist.angular.z = float(data.get("omega", 0.0))

            self.cmd_vel_publisher.publish(twist)
            # self.get_logger().debug(f"Published cmd_vel: {twist}")

        except json.JSONDecodeError:
            self.get_logger().warning(f"Invalid JSON received: {data_str}")
        except Exception as e:
            self.get_logger().error(f"Error processing bluetooth data: {e}")

    def on_odom_received(self, msg: Odometry):
        """Handle odometry data -> Send to Bluetooth client"""
        if self.ble_server and self.event_loop:
            try:
                # Create simple JSON for visualization
                # Note: sending minimal data to keep bandwidth low
                response = {
                    "type": "robot_pos",
                    "x": round(msg.pose.pose.position.x, 3),
                    "y": round(msg.pose.pose.position.y, 3),
                    # Simplified: using z-component of angular velocity or quaternion conversion if needed
                    # For visualization, position is most important
                }
                
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
