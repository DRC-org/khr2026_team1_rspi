import asyncio
import threading
from typing import Optional

from rclpy.node import Node
from std_msgs.msg import String

from .gatt_server import BluetoothGATTServer


class BluetoothROS2Node(Node):
    def __init__(self):
        super().__init__("bluetooth_node")

        # Declare parameters
        self.declare_parameter("hci_transport", "usb:0")
        self.declare_parameter("publish_interval", 0.1)  # seconds

        # Get parameters
        self.hci_transport: str = self.get_parameter("hci_transport").value or "usb:0"
        self.publish_interval: float = (
            self.get_parameter("publish_interval").value or 0.1
        )

        # Create publisher for receiving data from Bluetooth
        self.rx_publisher = self.create_publisher(String, "bluetooth_rx", 10)

        # Create subscriber for sending data via Bluetooth
        self.tx_subscriber = self.create_subscription(
            String, "bluetooth_tx", self.on_tx_message, 10
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

    def on_bluetooth_data_received(self, data: str):
        msg = String()
        msg.data = data
        self.rx_publisher.publish(msg)
        self.get_logger().info(f"Published Bluetooth RX: {data}")

    def on_tx_message(self, msg: String):
        if self.ble_server and self.event_loop:
            self.get_logger().info(f"Received TX message: {msg.data}")
            # Send data via Bluetooth (must use event loop thread)
            asyncio.run_coroutine_threadsafe(
                self.ble_server.send_data(msg.data), self.event_loop
            )

    def destroy_node(self):
        self.get_logger().info("Shutting down Bluetooth node")
        super().destroy_node()
