
import asyncio
import websockets
import json

async def verify_control():
    uri = "ws://localhost:9090"
    print(f"Connecting to {uri}...")
    
    async with websockets.connect(uri) as websocket:
        print("Connected. Subscribing to /cmd_vel...")
        
        # Subscribe to /cmd_vel
        subscribe_msg = {
            "op": "subscribe",
            "topic": "/cmd_vel",
            "type": "geometry_msgs/Twist"
        }
        await websocket.send(json.dumps(subscribe_msg))
        
        print("Listening for control commands...")
        count = 0
        try:
            while count < 5: # Listen for 5 valid messages
                message = await websocket.recv()
                data = json.loads(message)
                
                if data.get("op") == "publish" and data.get("topic") == "/cmd_vel":
                    linear = data["msg"]["linear"]
                    angular = data["msg"]["angular"]
                    print(f"✅ Received Control Command: Linear_X={linear['x']:.2f}, Angular_Z={angular['z']:.2f}")
                    
                    if linear['x'] > 0:
                         print("   -> Robot is MOVING")
                    else:
                         print("   -> Robot is STOPPED")
                    
                    count += 1
        except Exception as e:
            print(f"Error: {e}")

if __name__ == "__main__":
    try:
        asyncio.run(verify_control())
    except KeyboardInterrupt:
        pass
