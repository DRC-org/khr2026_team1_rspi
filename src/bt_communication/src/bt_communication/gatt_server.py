import asyncio
import json
import logging
import math
import random
from typing import Optional

from bumble.att import Attribute
from bumble.core import AdvertisingData
from bumble.device import Device
from bumble.gatt import Characteristic, CharacteristicValue, Service
from bumble.hci import Address
from bumble.transport import open_transport

from .constants import RX_CHARACTERISTIC_UUID, SERVICE_UUID, TX_CHARACTERISTIC_UUID


class BluetoothGATTServer:
    def __init__(self, on_data_received=None):
        self.device: Optional[Device] = None
        self.tx_char: Optional[Characteristic] = None
        self.send_task: Optional[asyncio.Task] = None
        self.on_data_received = on_data_received
        self.logger = logging.getLogger("BluetoothGATTServer")
        self.connected_clients = {}  # Connection -> properties dict

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
                    try:
                        data = json.loads(text)
                        if (
                            isinstance(data, dict)
                            and connection in self.connected_clients
                        ):
                            c_type = data.get("controller_type")
                            if c_type:
                                self.connected_clients[connection]["type"] = c_type
                                self.logger.info(
                                    f"Client {connection} type set to: {c_type}"
                                )
                    except json.JSONDecodeError:
                        pass

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

            # Create advertising data
            advertising_data = AdvertisingData(
                [(AdvertisingData.COMPLETE_LOCAL_NAME, bytes(target_name, "utf-8"))]
            )

            # Connection handlers
            async def on_connection(connection):
                self.logger.info(f"Client connected: {connection}")
                self.connected_clients[connection] = {}

                def on_disconnect(reason):
                    self.logger.info(
                        f"Client disconnected: {connection} reason: {reason}"
                    )
                    if connection in self.connected_clients:
                        del self.connected_clients[connection]

                    if not self.connected_clients:
                        if self.send_task and not self.send_task.done():
                            self.send_task.cancel()
                        self.send_task = None

                connection.on("disconnection", on_disconnect)

                if self.send_task is None or self.send_task.done():
                    self.send_task = asyncio.create_task(
                        self._send_messages_periodically()
                    )

                # 2 台目以降も接続できるようにアドバタイズを再開
                if self.device:
                    await self.device.start_advertising(
                        advertising_data=bytes(advertising_data), auto_restart=True
                    )

            self.device.on("connection", on_connection)

            self.logger.info(f"Server started: {self.device.name}")
            await self.device.power_on()

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
                    # type が controller のクライアントにのみ送信
                    for connection, props in list(self.connected_clients.items()):
                        if props.get("type") == "controller":
                            try:
                                await self.device.notify_subscriber(
                                    connection, self.tx_char, message
                                )
                            except Exception as e:
                                self.logger.warning(
                                    f"Failed to send to {connection}: {e}"
                                )

                # self.logger.info(f"Sent: {message.decode('utf-8')}")
                await asyncio.sleep(0.1)  # 100ms interval
        except asyncio.CancelledError:
            self.logger.info("Stopped sending messages")
            raise

    async def send_data(self, data: str):
        # TODO: 送信するデバイスタイプを選択できるようにする
        if self.tx_char and self.device:
            self.tx_char.value = data.encode("utf-8")
            await self.device.notify_subscribers(self.tx_char)
            self.logger.info(f"Sent to client: {data}")
