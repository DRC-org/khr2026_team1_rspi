import asyncio
import threading

import aiohttp.web
from rclpy.node import Node
from std_msgs.msg import String


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
        self._clients: set[aiohttp.web.WebSocketResponse] = set()
        self._clients_lock = threading.Lock()

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
        asyncio.run_coroutine_threadsafe(self._broadcast(msg.data), self._loop)

    async def _broadcast(self, data: str) -> None:
        with self._clients_lock:
            clients = list(self._clients)

        dead: list[aiohttp.web.WebSocketResponse] = []
        for ws in clients:
            try:
                await ws.send_str(data)
            except Exception:
                dead.append(ws)

        if dead:
            with self._clients_lock:
                for ws in dead:
                    self._clients.discard(ws)

    async def _ws_handler(self, request: aiohttp.web.Request) -> aiohttp.web.WebSocketResponse:
        ws = aiohttp.web.WebSocketResponse()
        await ws.prepare(request)

        with self._clients_lock:
            self._clients.add(ws)
        self.get_logger().info(f"WebSocket client connected: {request.remote}")

        try:
            async for msg in ws:
                if msg.type == aiohttp.WSMsgType.TEXT:
                    self._pub.publish(String(data=msg.data))
                elif msg.type in (aiohttp.WSMsgType.ERROR, aiohttp.WSMsgType.CLOSE):
                    break
        finally:
            with self._clients_lock:
                self._clients.discard(ws)
            self.get_logger().info(f"WebSocket client disconnected: {request.remote}")

        return ws

    def _run_server(self) -> None:
        asyncio.set_event_loop(self._loop)
        app = aiohttp.web.Application()
        app.router.add_get("/ws", self._ws_handler)
        runner = aiohttp.web.AppRunner(app)
        self._loop.run_until_complete(runner.setup())
        site = aiohttp.web.TCPSite(runner, "0.0.0.0", 8080)
        self._loop.run_until_complete(site.start())
        self._loop.run_forever()


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = WebServerNode()
    # MultiThreadedExecutor を使用: aiohttp の Waitable が SingleThreadedExecutor の
    # コールバックをブロックする問題を回避するため（routing_node と同じ理由）
    executor = MultiThreadedExecutor()
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
