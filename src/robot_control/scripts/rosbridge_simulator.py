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
                print("[Simulator] Computing new path...")
                # 計算時間を延ばして、より良い解（無駄のないルート）を探させる
                path_ids = self.optimizer.solve(start_node=0, time_limit=60)
                
                if path_ids:
                    # Prepare path data
                    self.current_path = {
                        "spots": self.optimizer.spots,
                        "path": path_ids,
                        "timestamp": time.time()
                    }
                    
                    path_names = [self.optimizer.spots[pid]["name"] for pid in path_ids]
                    print(f"[Simulator] Computed path: {path_names}")
                    
                    # Broadcast to all clients
                    await self.send_path_update()
                else:
                    print("[Simulator] No valid path found")
                
            except Exception as e:
                print(f"[Simulator] Error computing path: {e}")
            
            # Wait before next computation
            await asyncio.sleep(10)
    
    async def pose_publisher(self):
        """Simulate robot movement and publish pose"""
        print("Pose publisher started.")
        import math
        
        # State
        current_pos = [0.0, 0.0]
        current_path_ids = []
        target_idx = 0
        speed = 2.1 # m/s
        
        dt = 0.05 # 20Hz
        
        while True:
            # Check if path updated
            if self.current_path:
                new_path_ids = self.current_path['path']
                
                # If path changed drastically or reset needed
                if new_path_ids != current_path_ids:
                    current_path_ids = new_path_ids
                    spots = self.current_path['spots']
                    # Start from the first node (Start)
                    start_id = current_path_ids[0]
                    current_pos = [spots[start_id]['x'], spots[start_id]['y']]
                    target_idx = 1 # Aim for next node
                    
                    # Update speed from optimizer params if available
                    speed = self.optimizer.params.get("robot_speed", 2.1)
            
            # Move robot
            if current_path_ids and target_idx < len(current_path_ids):
                spots = self.current_path['spots']
                target_id = current_path_ids[target_idx]
                target = spots[target_id]
                tx, ty = target['x'], target['y']
                
                cx, cy = current_pos
                dx = tx - cx
                dy = ty - cy
                dist = math.sqrt(dx*dx + dy*dy)
                
                step = speed * dt
                
                if dist < step:
                    # Reached target
                    current_pos = [tx, ty]
                    target_idx += 1
                    # Simulate action wait? (Skip for smooth viz)
                else:
                    # Move towards target
                    vx = (dx / dist) * speed
                    vy = (dy / dist) * speed
                    current_pos[0] += vx * dt
                    current_pos[1] += vy * dt
            
            # Publish Pose
            msg = {
                "op": "publish",
                "topic": "/robot/pose",
                "msg": {
                    "header": {
                        "frame_id": "map",
                        "stamp": time.time()
                    },
                    "pose": {
                        "position": {"x": current_pos[0], "y": current_pos[1], "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                    }
                }
            }
            
            # Publish Twist (cmd_vel) simulation
            # Moving if remaining distance > step
            current_speed = 0.0
            if current_path_ids and target_idx < len(current_path_ids):
                 # Recalculate distance to check if moving
                 spots = self.current_path['spots']
                 target = spots[current_path_ids[target_idx]]
                 dx = target['x'] - current_pos[0]
                 dy = target['y'] - current_pos[1]
                 if math.sqrt(dx*dx + dy*dy) > speed * dt:
                     current_speed = speed

            twist_msg = {
                "op": "publish",
                "topic": "/cmd_vel",
                "msg": {
                    "linear": { "x": current_speed, "y": 0.0, "z": 0.0 },
                    "angular": { "x": 0.0, "y": 0.0, "z": 0.0 }
                }
            }
            
            message_str = json.dumps(msg)
            twist_str = json.dumps(twist_msg)
            
            # Broadcast to all clients
            if self.clients:
                # Use a copy of set to avoid runtime error if size changes
                clients_copy = self.clients.copy()
                if clients_copy:
                    await asyncio.gather(
                        *[client.send(message_str) for client in clients_copy],
                        *[client.send(twist_str) for client in clients_copy],
                        return_exceptions=True
                    )
            
            await asyncio.sleep(dt)

    async def start(self):
        """Start the WebSocket server"""
        print(f"Starting ROS Bridge Simulator on ws://{self.host}:{self.port}")
        print("Waiting for React UI to connect...")
        
        # Start path publisher task
        asyncio.create_task(self.path_publisher())
        
        # Start pose simulation task
        asyncio.create_task(self.pose_publisher())
        
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
