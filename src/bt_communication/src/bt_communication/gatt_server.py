import asyncio
import json
import logging
import random
from typing import Optional

from bumble.att import Attribute
from bumble.core import AdvertisingData
from bumble.device import (
    AdvertisingEventProperties,
    AdvertisingParameters,
    Device,
)
from bumble.gatt import Characteristic, CharacteristicValue, Service
from bumble.hci import Address, OwnAddressType
from bumble.transport import open_transport

from .constants import RX_CHARACTERISTIC_UUID, SERVICE_UUID, TX_CHARACTERISTIC_UUID


class BluetoothGATTServer:
    def __init__(self, on_data_received=None, publish_interval: float = 0.2):
        self.device: Optional[Device] = None
        self.tx_char: Optional[Characteristic] = None
        self.send_task: Optional[asyncio.Task] = None
        self.on_data_received = on_data_received
        self.publish_interval = publish_interval
        self.logger = logging.getLogger("BluetoothGATTServer")
        self.connected_clients = {}  # Connection -> properties dict
        # Buffer for pending TX data (set from ROS thread via call_soon_threadsafe)
        self._pending_tx_data: Optional[str] = None

    def set_pending_tx_data(self, data: str):
        """Store latest TX data. Called on event loop via call_soon_threadsafe."""
        self._pending_tx_data = data

    async def _start_advertising(self):
        """Start or restart advertising with max TX power."""
        await self.device.stop_advertising()
        adv_set = await self.device.create_advertising_set(
            advertising_parameters=AdvertisingParameters(
                advertising_event_properties=(
                    AdvertisingEventProperties(
                        is_connectable=True,
                        is_scannable=True,
                        is_legacy=True,
                    )
                ),
                own_address_type=OwnAddressType.RANDOM,
                advertising_tx_power=29,
            ),
            random_address=self.device.random_address,
            advertising_data=self._advertising_data_bytes,
            auto_start=True,
            auto_restart=True,
        )
        self.logger.info(
            f"Advertising started (selected TX power: "
            f"{adv_set.selected_tx_power} dBm)"
        )

    async def start(self, hci_transport_uri: str = "usb:2357:0604/E848B8C82000"):
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
                    self.logger.debug(f"Received write from client: {text}")
                    try:
                        data = json.loads(text)
                        if (
                            isinstance(data, dict)
                            and connection in self.connected_clients
                        ):
                            c_type = data.get("controller_type")
                            if c_type:
                                self.connected_clients[connection]["type"] = c_type
                                self.logger.debug(
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

            # Store advertising data for reuse
            self._advertising_data_bytes = bytes(
                AdvertisingData(
                    [
                        (
                            AdvertisingData.COMPLETE_LOCAL_NAME,
                            bytes(target_name, "utf-8"),
                        )
                    ]
                )
            )

            # Connection handlers
            async def on_connection(connection):
                self.logger.info(f"Client connected: {connection}")
                self.connected_clients[connection] = {}

                # 混線環境対策: supervision timeout を延長し、
                # 一時的な干渉でもすぐに切断されないようにする
                try:
                    await connection.update_connection_parameters(
                        conn_interval_min=24,   # 30ms (24 * 1.25ms)
                        conn_interval_max=40,   # 50ms (40 * 1.25ms)
                        conn_latency=0,
                        supervision_timeout=500, # 5000ms (500 * 10ms)
                    )
                    self.logger.info(
                        "Connection parameters updated "
                        "(interval=30-50ms, supervision_timeout=5000ms)"
                    )
                except Exception as e:
                    self.logger.warning(
                        f"Failed to update connection parameters: {e}"
                    )

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
                    try:
                        await self._start_advertising()
                    except Exception as e:
                        self.logger.warning(
                            f"Failed to restart advertising after connection: {e}"
                        )

            self.device.on("connection", on_connection)

            self.logger.info(f"Server started: {self.device.name}")
            await self.device.power_on()

            await self._start_advertising()

            self.logger.info("Waiting for connections")

            # Keep server running
            await asyncio.get_running_loop().create_future()

    async def _send_messages_periodically(self):
        """Send pending TX data at fixed intervals.

        Uses wall-clock scheduling so that notify_subscribers() latency
        does not add to the interval.
        """
        interval = self.publish_interval
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
                        # Fragmentation: split into 240-byte chunks
                        # (safe margin below 247-byte ATT MTU common on Linux BLE)
                        # The web controller (receiver) reassembles {} objects via
                        # brace-depth counting in rxBuffer.
                        data_bytes = pending.encode("utf-8")
                        chunk_size = 240
                        for i in range(0, len(data_bytes), chunk_size):
                            chunk = data_bytes[i : i + chunk_size]
                            self.tx_char.value = chunk
                            t0 = loop.time()
                            try:
                                await self.device.notify_subscribers(self.tx_char)
                            except Exception as e:
                                self.logger.warning(
                                    f"notify_subscribers failed (chunk {i//chunk_size}): {e}"
                                )
                                break  # 残チャンクを送らずタスクは継続
                            notify_time = loop.time() - t0
                            _dbg_count += 1
                            _dbg_total_notify += notify_time
                            if notify_time > _dbg_max_notify:
                                _dbg_max_notify = notify_time

                # === DEBUG: log every 3 seconds ===
                now = loop.time()
                if now - _dbg_last_log > 3.0:
                    if _dbg_count > 0:
                        self.logger.debug(
                            f"[BLE TX DEBUG] {_dbg_count} chunks in 3s "
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
            self.logger.debug(f"Sent to client: {data}")
