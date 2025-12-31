import asyncio
import json
import logging
import math
import random
import threading
from typing import Optional

import rclpy
from bumble.att import Attribute
from bumble.core import AdvertisingData
from bumble.device import Device
from bumble.gatt import Characteristic, CharacteristicValue, Service
from bumble.hci import Address
from bumble.transport import open_transport
from rclpy.node import Node
from std_msgs.msg import String

logging.basicConfig(level=logging.INFO)

SERVICE_UUID = "845d1d9a-b986-45b8-8b0e-21ee94307983"
TX_CHARACTERISTIC_UUID = "3ecd3272-0f80-4518-ad58-78aa9af3ec9d"
RX_CHARACTERISTIC_UUID = "47153006-9eef-45e5-afb7-038ea60ad893"


class BluetoothGATTServer:
    """Bluetooth GATT Server using Bumble"""

    def __init__(self, on_data_received=None):
        self.device: Optional[Device] = None
        self.tx_char: Optional[Characteristic] = None
        self.send_task: Optional[asyncio.Task] = None
        self.on_data_received = on_data_received
        self.logger = logging.getLogger("BluetoothGATTServer")

    async def start(self, hci_transport_uri: str = "usb:0"):
        """Start the Bluetooth GATT server"""
        self.logger.info("Starting BLE GATT Server...")
        self.logger.info(f"Connecting to HCI transport: {hci_transport_uri}")

        async with await open_transport(hci_transport_uri) as hci_transport:
            self.logger.info("Connected to HCI transport")

            # Generate a random device name
            target_name = f"Hojicha_{random.randint(0, 9999):04d}"

            # Generate a static random address
            def make_static_random_address():
                bytes_ = bytearray(random.getrandbits(8) for _ in range(6))
                bytes_[0] = (bytes_[0] & 0x3F) | 0xC0
                return ":".join(f"{b:02X}" for b in bytes_)

            rand_addr = make_static_random_address()
            self.logger.info(f"Using address: {rand_addr}")

            self.device = Device.with_hci(
                target_name,
                Address(rand_addr),
                hci_transport.source,
                hci_transport.sink,
            )

            # RX write handler
            def on_rx_write(connection, value):
                try:
                    text = value.decode("utf-8")
                    self.logger.info(f"Received write from client: {text}")
                    if self.on_data_received:
                        self.on_data_received(text)
                except Exception as e:
                    self.logger.warning(
                        f"Failed to decode received data: {value.hex()}, {e}"
                    )

            # Define characteristics
            self.tx_char = Characteristic(
                TX_CHARACTERISTIC_UUID,
                properties=(
                    Characteristic.Properties.READ | Characteristic.Properties.NOTIFY
                ),
                permissions=(Attribute.Permissions.READABLE),
                value=b"Hello from ROS2 Bluetooth!",
            )

            rx_char = Characteristic(
                RX_CHARACTERISTIC_UUID,
                properties=(
                    Characteristic.Properties.WRITE
                    | Characteristic.Properties.WRITE_WITHOUT_RESPONSE
                ),
                permissions=(Attribute.Permissions.WRITEABLE),
                value=CharacteristicValue(write=on_rx_write),
            )

            # Define service
            service = Service(SERVICE_UUID, [self.tx_char, rx_char])
            self.device.add_service(service)

            # Connection handlers
            def on_connection(connection):
                self.logger.info(f"Client connected: {connection}")
                connection.on("disconnection", on_disconnection)
                self.send_task = asyncio.create_task(self._send_messages_periodically())

            def on_disconnection(connection):
                self.logger.info(f"Client disconnected: {connection}")
                if self.send_task and not self.send_task.done():
                    self.send_task.cancel()
                self.send_task = None

            self.device.on("connection", on_connection)
            self.device.on("disconnection", on_disconnection)

            self.logger.info(f"Server started: {self.device.name}")
            await self.device.power_on()

            # Create advertising data
            advertising_data = AdvertisingData(
                [(AdvertisingData.COMPLETE_LOCAL_NAME, bytes(target_name, "utf-8"))]
            )

            await self.device.start_advertising(
                advertising_data=bytes(advertising_data), auto_restart=True
            )

            self.logger.info("Advertising... Waiting for connections")

            # Keep server running
            await asyncio.get_running_loop().create_future()

    async def _send_messages_periodically(self):
        """Send robot position data every 100ms"""
        # Define waypoints for smooth path
        waypoints = [
            (388, 388),
            (388, 6500),
            (1100, 6500),
            (1100, 1400),
            (1900, 1400),
            (1900, 6500),
            (2500, 6500),
            (2500, 388),
            (388, 388),
        ]

        def distance(p1, p2):
            return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)

        def interpolate_path(waypoints, progress):
            """Interpolate position along waypoints. progress: 0.0 to 1.0"""
            distances = [0.0]
            for i in range(len(waypoints) - 1):
                distances.append(
                    distances[-1] + distance(waypoints[i], waypoints[i + 1])
                )

            total_distance = distances[-1]
            target_distance = progress * total_distance

            for i in range(len(distances) - 1):
                if distances[i] <= target_distance <= distances[i + 1]:
                    segment_progress = (
                        (target_distance - distances[i])
                        / (distances[i + 1] - distances[i])
                        if distances[i + 1] != distances[i]
                        else 0
                    )
                    p1 = waypoints[i]
                    p2 = waypoints[i + 1]
                    x = p1[0] + segment_progress * (p2[0] - p1[0])
                    y = p1[1] + segment_progress * (p2[1] - p1[1])
                    return x, y

            return waypoints[-1]

        message_counter = 0
        try:
            while True:
                message_counter += 1

                # Calculate position along the path
                progress = (message_counter % 200) / 200.0
                x, y = interpolate_path(waypoints, progress)

                # Calculate angle
                angle = 15 * math.sin(2 * math.pi * message_counter / 40)

                # Create JSON message
                data = {
                    "type": "robot_pos",
                    "x": round(x, 2),
                    "y": round(y, 2),
                    "angle": round(angle, 2),
                }
                message = json.dumps(data).encode("utf-8")
                if self.tx_char:
                    self.tx_char.value = message
                if self.device and self.tx_char:
                    await self.device.notify_subscribers(self.tx_char)
                self.logger.info(f"Sent: {message.decode('utf-8')}")
                await asyncio.sleep(0.1)  # 100ms interval
        except asyncio.CancelledError:
            self.logger.info("Stopped sending messages")
            raise

    async def send_data(self, data: str):
        """Send data to connected clients"""
        if self.tx_char and self.device:
            self.tx_char.value = data.encode("utf-8")
            await self.device.notify_subscribers(self.tx_char)
            self.logger.info(f"Sent to client: {data}")


class BluetoothROS2Node(Node):
    """ROS2 Node for Bluetooth GATT Server"""

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
        """Callback when data is received from Bluetooth"""
        msg = String()
        msg.data = data
        self.rx_publisher.publish(msg)
        self.get_logger().info(f"Published Bluetooth RX: {data}")

    def on_tx_message(self, msg: String):
        """Callback when receiving TX message from ROS2"""
        if self.ble_server and self.event_loop:
            self.get_logger().info(f"Received TX message: {msg.data}")
            # Send data via Bluetooth (must use event loop thread)
            asyncio.run_coroutine_threadsafe(
                self.ble_server.send_data(msg.data), self.event_loop
            )

    def destroy_node(self):
        """Cleanup on node shutdown"""
        self.get_logger().info("Shutting down Bluetooth node")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = BluetoothROS2Node()
    node.start_bluetooth_server()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
