#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import asyncio
import websockets
import json
import threading

class WebsocketBroadcaster(Node):
	def __init__(self):
		super().__init__('websocket_broadcaster')
		
		# Format the data as a JSON dictionary
		self.payload = {
			"waypoints": [
				{"x": -0.579, "y": 0.000, "yaw": 0.0000},
	{"x": -0.565, "y": -0.512, "yaw": -1.5433},
	{"x": -1.099, "y": -1.681, "yaw": -2.1800},
	{"x": -1.554, "y": -4.119, "yaw": -1.7558},
	{"x": -0.929, "y": -4.268, "yaw": -0.2346},
	{"x": 1.028, "y": -4.344, "yaw": -0.0388},
	{"x": 2.229, "y": -4.118, "yaw": 0.1867},
	{"x": 4.522, "y": -4.275, "yaw": 0.0778},
	{"x": 4.614, "y": -3.967, "yaw": 1.2806},
	{"x": 4.595, "y": -1.987, "yaw": 1.5801},
	{"x": 4.353, "y": -1.179, "yaw": 1.8625},

	# Your newly calculated points
	{"x": 4.780, "y": 1.859, "yaw": 2.237},
	{"x": 3.811, "y": 2.610, "yaw": 2.272},
			]
		}
		self.get_logger().info('Starting WebSocket server on ws://0.0.0.0:8765')

	async def broadcast_handler(self, websocket):
		"""Continuously sends the JSON data to any connected client"""
		try:
			while True:
				# Convert dict to JSON string and send
				await websocket.send(json.dumps(self.payload))
				# Broadcast every 2 seconds
				await asyncio.sleep(2.0)
		except websockets.exceptions.ConnectionClosed:
			self.get_logger().info("A client disconnected.")

	def start_server(self):
		"""Starts the asyncio event loop for the websocket server"""
		loop = asyncio.new_event_loop()
		asyncio.set_event_loop(loop)

		async def run_server():
			async with websockets.serve(self.broadcast_handler, "0.0.0.0", 8765):
				await asyncio.Future()  # run forever

		loop.run_until_complete(run_server())

def main(args=None):
	rclpy.init(args=args)
	node = WebsocketBroadcaster()
	
	# Run the WebSocket server in a separate daemon thread
	# This allows rclpy to spin normally in the main thread if needed later
	ws_thread = threading.Thread(target=node.start_server, daemon=True)
	ws_thread.start()
	
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()
