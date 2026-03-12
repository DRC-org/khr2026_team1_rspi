import asyncio
import os
import threading
from pathlib import Path

import aiohttp.web
from rclpy.node import Node
from std_msgs.msg import String

_DEFAULT_DIST = "/home/taiga/DRC/khr2026_team1_bt_controller/dist"


class WebServerNode(Node):
    """
    WebSocket ブリッジノード。

    aiohttp WebSocket サーバー（ポート 8080）を別スレッドの asyncio ループで起動し、
    ROS2 の bluetooth_rx / bluetooth_tx トピックと双方向にブリッジする:
      - WS クライアント → bluetooth_rx（ブラウザからのコマンドを ROS2 に流す）
      - bluetooth_tx → WS クライアント（rspi からの状態通知をブラウザに配信）
    """

    def __init__(self):
        super().__init__("web_server_node")
        self._ws_clients: set[aiohttp.web.WebSocketResponse] = set()
        self._ws_clients_lock = threading.Lock()

        self._pub = self.create_publisher(String, "bluetooth_rx", 10)
        self._sub = self.create_subscription(
            String, "bluetooth_tx", self._on_bluetooth_tx, 10
        )

        self._loop = asyncio.new_event_loop()
        self._server_thread = threading.Thread(
            target=self._run_server, daemon=True
        )
        self._server_thread.start()
        self.get_logger().info("WebServerNode started (ws://0.0.0.0:8080/ws)")

    def _on_bluetooth_tx(self, msg: String) -> None:
        with self._ws_clients_lock:
            if not self._ws_clients:
                return
        asyncio.run_coroutine_threadsafe(self._broadcast(msg.data), self._loop)

    async def _broadcast(self, data: str) -> None:
        with self._ws_clients_lock:
            clients = list(self._ws_clients)

        dead: list[aiohttp.web.WebSocketResponse] = []
        for ws in clients:
            try:
                await ws.send_str(data)
            except Exception:
                dead.append(ws)

        if dead:
            with self._ws_clients_lock:
                for ws in dead:
                    self._ws_clients.discard(ws)

    async def _ws_handler(self, request: aiohttp.web.Request) -> aiohttp.web.WebSocketResponse:
        ws = aiohttp.web.WebSocketResponse()
        await ws.prepare(request)

        with self._ws_clients_lock:
            self._ws_clients.add(ws)
        self.get_logger().info(f"WebSocket client connected: {request.remote}")

        try:
            async for msg in ws:
                if msg.type == aiohttp.WSMsgType.TEXT:
                    self.get_logger().debug(f"WS recv: {msg.data[:100]}")
                    self._pub.publish(String(data=msg.data))
                elif msg.type in (aiohttp.WSMsgType.ERROR, aiohttp.WSMsgType.CLOSE):
                    self.get_logger().info(f"WS msg type={msg.type}, closing")
                    break
        except Exception as e:
            self.get_logger().error(f"WS handler error: {e}")
        finally:
            with self._ws_clients_lock:
                self._ws_clients.discard(ws)
            self.get_logger().info(f"WebSocket client disconnected: {request.remote}")

        return ws

    async def _index_handler(self, _request: aiohttp.web.Request) -> aiohttp.web.FileResponse:
        return aiohttp.web.FileResponse(self._dist_path / "index.html")

    def _run_server(self) -> None:
        asyncio.set_event_loop(self._loop)

        self._dist_path = Path(
            os.environ.get("DRC_CONTROLLER_DIST", _DEFAULT_DIST)
        )

        app = aiohttp.web.Application()
        app.router.add_get("/ws", self._ws_handler)

        if self._dist_path.is_dir():
            assets_path = self._dist_path / "assets"
            if assets_path.is_dir():
                app.router.add_static("/assets/", assets_path)
            app.router.add_get("/", self._index_handler)
            self.get_logger().info(f"Serving static files from {self._dist_path}")
        else:
            self.get_logger().warn(
                f"dist directory not found: {self._dist_path} — static file serving disabled"
            )

        runner = aiohttp.web.AppRunner(app)
        self._loop.run_until_complete(runner.setup())
        site = aiohttp.web.TCPSite(runner, "0.0.0.0", 8080)
        self._loop.run_until_complete(site.start())
        self._loop.run_forever()


def main(args=None):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    rclpy.init(args=args)
    node = WebServerNode()
    # aiohttp は独立したスレッド・asyncio ループで動作し ROS2 Executor に Waitable を
    # 登録しないため、SingleThreadedExecutor で十分
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
