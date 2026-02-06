#!/usr/bin/env python3
"""
Simple ROS-bridge compatible WebSocket server for path visualization.
Runs without ROS 2 installation - standalone simulation mode.
"""

import asyncio
import json
import websockets
import time
from pathlib import Path
import sys

# Add parent directory to path for task_optimizer import
sys.path.insert(0, str(Path(__file__).parent))

try:
    from task_optimizer_standalone import TaskOptimizer
except ImportError as e:
    print(f"Error importing TaskOptimizer: {e}")
    print("Make sure ortools is installed: pip install ortools --break-system-packages")
    sys.exit(1)


class ROSBridgeSimulator:
    def __init__(self, host='localhost', port=9090):
        self.host = host
        self.port = port
        self.clients = set()
        self.optimizer = TaskOptimizer()
        self.current_path = None
        
    async def register(self, websocket):
        """Register a new client connection"""
        self.clients.add(websocket)
        print(f"Client connected. Total clients: {len(self.clients)}")
        
        # Send current path if available
        if self.current_path:
            await self.send_path_update(websocket)
    
    async def unregister(self, websocket):
        """Unregister a client connection"""
        self.clients.discard(websocket)
        print(f"Client disconnected. Total clients: {len(self.clients)}")
    
    async def send_path_update(self, websocket=None):
        """Send path update to client(s)"""
        if not self.current_path:
            return
        
        # Format as ROS bridge message
        message = {
            "op": "publish",
            "topic": "/task_planner/planned_path",
            "msg": {
                "data": json.dumps(self.current_path)
            }
        }
        
        message_str = json.dumps(message)
        
        if websocket:
            # Send to specific client
            try:
                await websocket.send(message_str)
            except websockets.exceptions.ConnectionClosed:
                pass
        else:
            # Broadcast to all clients
            if self.clients:
                await asyncio.gather(
                    *[client.send(message_str) for client in self.clients],
                    return_exceptions=True
                )
    
    async def handle_client_message(self, websocket, message_str):
        """Handle incoming messages from clients"""
        try:
            message = json.loads(message_str)
            op = message.get('op')
            
            if op == 'subscribe':
                topic = message.get('topic')
                print(f"Client subscribed to: {topic}")
                
                # Send current path immediately if subscribing to path topic
                if topic == '/task_planner/planned_path' and self.current_path:
                    await self.send_path_update(websocket)
            
            elif op == 'unsubscribe':
                topic = message.get('topic')
                print(f"Client unsubscribed from: {topic}")
            
            elif op == 'advertise':
                topic = message.get('topic')
                print(f"Client advertising: {topic}")
        
        except json.JSONDecodeError:
            print("Received invalid JSON from client")
        except Exception as e:
            print(f"Error handling client message: {e}")
    
    async def client_handler(self, websocket):
        """Handle a single client connection"""
        client_addr = websocket.remote_address
        print(f"[Server] New connection from {client_addr}")
        
        await self.register(websocket)
        
        try:
            async for message in websocket:
                print(f"[Server] Received message from {client_addr}: {message[:200]}...")
                await self.handle_client_message(websocket, message)
        except websockets.exceptions.ConnectionClosed as e:
            print(f"[Server] Connection closed from {client_addr}: {e}")
        except Exception as e:
            print(f"[Server] Error handling client {client_addr}: {e}")
        finally:
            await self.unregister(websocket)
    
    async def path_publisher(self):
        """Periodically compute and publish new paths"""
        print("Path publisher started. Will compute paths every 10 seconds...")
        
        while True:
            try:
                # Compute new path
                print("\n[Simulator] Computing new path...")
                path_ids = self.optimizer.solve(start_node=0, time_limit=180)
                
                if path_ids:
                    # Prepare path data
                    self.current_path = {
                        "spots": self.optimizer.spots,
                        "path": path_ids,
                        "timestamp": time.time()
                    }
                    
                    print(f"[Simulator] Computed path: {[self.optimizer.spots[pid]['name'] for pid in path_ids]}")
                    
                    # Broadcast to all clients
                    await self.send_path_update()
                else:
                    print("[Simulator] No valid path found")
                
            except Exception as e:
                print(f"[Simulator] Error computing path: {e}")
            
            # Wait before next computation
            await asyncio.sleep(10)
    
    async def start(self):
        """Start the WebSocket server"""
        print(f"Starting ROS Bridge Simulator on ws://{self.host}:{self.port}")
        print("Waiting for React UI to connect...")
        
        # Start path publisher task
        asyncio.create_task(self.path_publisher())
        
        # Start WebSocket server
        async with websockets.serve(
            self.client_handler,
            self.host,
            self.port,
            ping_interval=20,
            ping_timeout=20
        ):
            await asyncio.Future()  # Run forever


def main():
    simulator = ROSBridgeSimulator(host='0.0.0.0', port=9090)
    
    try:
        asyncio.run(simulator.start())
    except KeyboardInterrupt:
        print("\nShutting down simulator...")


if __name__ == "__main__":
    main()
