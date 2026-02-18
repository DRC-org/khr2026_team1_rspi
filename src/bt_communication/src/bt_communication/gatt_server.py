import asyncio
import json
import logging
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
        # Buffer for pending TX data (set from ROS thread via call_soon_threadsafe)
        self._pending_tx_data: Optional[str] = None

    def set_pending_tx_data(self, data: str):
        """Store latest TX data. Called on event loop via call_soon_threadsafe."""
        self._pending_tx_data = data

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
        """Send pending TX data at fixed 100ms intervals.

        Uses wall-clock scheduling so that notify_subscribers() latency
        does not add to the interval (previous bug: sleep was ADDED to
        notify time, giving ~200ms cycles instead of 100ms).
        """
        interval = 0.1  # 100ms target
        loop = asyncio.get_event_loop()
        next_tick = loop.time() + interval
        # === DEBUG ===
        _dbg_count = 0
        _dbg_total_notify = 0.0
        _dbg_max_notify = 0.0
        _dbg_last_log = loop.time()

        try:
            while True:
                pending = self._pending_tx_data
                if pending is not None:
                    self._pending_tx_data = None
                    if self.tx_char and self.device:
                        encoded = pending.encode("utf-8")
                        self.tx_char.value = encoded
                        t0 = loop.time()
                        await self.device.notify_subscribers(self.tx_char)
                        notify_time = loop.time() - t0
                        _dbg_count += 1
                        _dbg_total_notify += notify_time
                        if notify_time > _dbg_max_notify:
                            _dbg_max_notify = notify_time

                # === DEBUG: log every 3 seconds ===
                now = loop.time()
                if now - _dbg_last_log > 3.0:
                    if _dbg_count > 0:
                        self.logger.info(
                            f"[BLE TX DEBUG] {_dbg_count} notifs in 3s "
                            f"({_dbg_count / 3:.1f}/s) | "
                            f"notify avg={_dbg_total_notify / _dbg_count * 1000:.1f}ms "
                            f"max={_dbg_max_notify * 1000:.1f}ms"
                        )
                    _dbg_count = 0
                    _dbg_total_notify = 0.0
                    _dbg_max_notify = 0.0
                    _dbg_last_log = now

                # Fixed-interval sleep: subtract elapsed time from interval
                now = loop.time()
                sleep_time = next_tick - now
                if sleep_time > 0:
                    await asyncio.sleep(sleep_time)
                next_tick += interval
                # If we fell behind (notify took too long), reset to prevent burst
                if next_tick < now:
                    next_tick = now + interval
        except asyncio.CancelledError:
            self.logger.info("Stopped sending messages")
            raise

    async def send_data(self, data: str):
        # TODO: 送信するデバイスタイプを選択できるようにする
        if self.tx_char and self.device:
            self.tx_char.value = data.encode("utf-8")
            await self.device.notify_subscribers(self.tx_char)
            self.logger.info(f"Sent to client: {data}")
