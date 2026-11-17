import asyncio
import json
import threading

import rclpy
import websockets

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from concurrent.futures import Future

import time
import glob
import os
import shutil
import psutil
from std_msgs.msg import String


class HoverRobotWebSocketBridge(Node):

    def __init__(self):
        super().__init__('hoverrobot_websocket_bridge')

        self.declare_parameter('topic_pointcloud', '/points2')
        self.declare_parameter('topic_telemetry', '/system/telemetry')
        self.declare_parameter('port', 8765)

        self.topicPointcloud = self.get_parameter('topic_pointcloud').value
        self.topicTelemetry = self.get_parameter('topic_telemetry').value
        self.port = self.get_parameter('port').value

        self.loop = None
        self.connected_clients = []
        self.sequence = 0

        self.subscription = self.create_subscription(
            PointCloud2,
            self.topicPointcloud,
            self.pointcloud_callback,
            qos_profile_sensor_data
        )

        self.telemetry_publisher = self.create_publisher(
            String,
            self.topicTelemetry,
            10
        )

        self.telemetry_timer = self.create_timer(
            0.5,
            self.publish_telemetry
        )
        
        self.get_logger().info(
            f'WebSocket Bridge initialized. Topic pointcloud: {self.topicPointcloud} Topic telemetry: {self.topicTelemetry}'
        )

    def pointcloud_callback(self, msg: PointCloud2):

        if not self.loop or not self.connected_clients:
            return

        self.sequence += 1

        packet = (self.sequence, msg)

        # Schedule delivery on the asyncio event loop.
        self.loop.call_soon_threadsafe(
            self.broadcast_latest,
            packet
        )

    def broadcast_latest(self, packet):

        # All client queues have a maximum size of one.
        # If a client is slow, discard its previous cloud.
        for client in list(self.connected_clients):

            queue = client['queue']

            if queue.full():
                try:
                    queue.get_nowait()
                except asyncio.QueueEmpty:
                    pass

            try:
                queue.put_nowait(packet)
            except asyncio.QueueFull:
                pass

    def get_cpu_temperature(self):
        try:
            with open('/sys/class/thermal/thermal_zone0/temp') as f:
                return round(int(f.read()) / 1000, 1)
        except (OSError, ValueError):
            return None


    def get_fan_speeds(self):
        fans = {}

        for path in glob.glob('/sys/class/hwmon/hwmon*/fan*_input'):
            try:
                name = os.path.basename(path).replace('_input', '')

                with open(path) as f:
                    fans[name] = int(f.read())

            except (OSError, ValueError):
                pass

        return fans


    def publish_telemetry(self):
        memory = psutil.virtual_memory()
        disk = shutil.disk_usage('/')

        telemetry = {
            'type': 'system_telemetry',
            'cpu': {
                'usage': psutil.cpu_percent(),
                'per_core': psutil.cpu_percent(percpu=True),
                'temp': self.get_cpu_temperature()
            },
            'fans': self.get_fan_speeds(),
            'memory': {
                'usage': memory.percent,
                'total_mb': round(memory.total / 1024**2),
                'available_mb': round(memory.available / 1024**2)
            },
            'disk': {
                'usage': round(disk.used / disk.total * 100, 1),
                'total_gb': round(disk.total / 1024**3, 1),
                'free_gb': round(disk.free / 1024**3, 1)
            },
            'uptime': round(time.time() - psutil.boot_time())
        }

        msg = String()
        msg.data = json.dumps(telemetry)

        self.telemetry_publisher.publish(msg)

        if self.loop and self.connected_clients:
            self.loop.call_soon_threadsafe(
                self.broadcast_telemetry,
                msg.data
            )


    def broadcast_telemetry(self, payload):
        for client in list(self.connected_clients):
            asyncio.create_task(
                self.send_telemetry(client, payload)
            )


    async def send_telemetry(self, client, payload):
        try:
            async with client['lock']:
                await client['websocket'].send(payload)

        except websockets.exceptions.ConnectionClosed:
            pass

    async def client_handler(self, websocket):

        client = {
            'websocket': websocket,
            'queue': asyncio.Queue(maxsize=1),
            'lock': asyncio.Lock()
        }

        self.connected_clients.append(client)

        self.get_logger().info(
            f'Client connected: {websocket.remote_address}'
        )

        try:
            while True:

                sequence, msg = await client['queue'].get()

                # First frame: PointCloud2 metadata.
                metadata = {
                    'type': 'pointcloud2',
                    'sequence': sequence,
                    'stamp': {
                        'sec': msg.header.stamp.sec,
                        'nanosec': msg.header.stamp.nanosec
                    },
                    'frame_id': msg.header.frame_id,
                    'height': msg.height,
                    'width': msg.width,
                    'fields': [
                        {
                            'name': field.name,
                            'offset': field.offset,
                            'datatype': field.datatype,
                            'count': field.count
                        }
                        for field in msg.fields
                    ],
                    'is_bigendian': msg.is_bigendian,
                    'point_step': msg.point_step,
                    'row_step': msg.row_step,
                    'is_dense': msg.is_dense,
                    'data_size': len(msg.data)
                }

                async with client['lock']:

                    await websocket.send(
                        json.dumps(metadata)
                    )

                    await websocket.send(
                        bytes(msg.data)
                    )

                self.get_logger().info(
                    f'Cloud sent: sequence={sequence}, bytes={len(msg.data)}'
                )

        except websockets.exceptions.ConnectionClosed as e:
            self.get_logger().warning(
                f'Client closed connection: code={e.code}, reason={e.reason}'
            )

        except Exception as e:
            self.get_logger().error(
                f'WebSocket client error: {type(e).__name__}: {e}'
            )
        finally:
            if client in self.connected_clients:
                self.connected_clients.remove(client)

            self.get_logger().info(
                'Client disconnected'
            )



async def run_server(node):

    node.loop = asyncio.get_running_loop()

    host = '0.0.0.0'

    async with websockets.serve(
        node.client_handler,
        host,
        node.port,
        compression=None,
        max_size=None,
        ping_interval=20,
        ping_timeout=20
    ):

        node.get_logger().info(
            f'WebSocket server listening on ws://{host}:{node.port}'
        )

        await asyncio.Future()


def main(args=None):

    rclpy.init(args=args)

    node = HoverRobotWebSocketBridge()

    # ROS executor runs independently from asyncio.
    ros_thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True
    )

    ros_thread.start()

    try:
        asyncio.run(run_server(node))

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        ros_thread.join(timeout=2)


if __name__ == '__main__':
    main()