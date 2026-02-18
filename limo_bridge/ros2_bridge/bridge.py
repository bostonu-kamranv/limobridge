import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import zmq
import json
import socket

# Get the hostname (e.g., "limo-01") and sanitize it for ROS (underscores only)
HOSTNAME = socket.gethostname().replace('-', '_')

class Ros2NamespaceBridge(Node):
    def __init__(self):
        super().__init__(f'{HOSTNAME}_bridge')
        
        self.get_logger().info(f"Starting Bridge for Namespace: /{HOSTNAME}")

        # --- ZMQ SETUP ---
        self.ctx = zmq.Context()
        
        # PUB: Send commands to ROS1 (Local)
        self.zmq_pub = self.ctx.socket(zmq.PUB)
        self.zmq_pub.bind("tcp://*:5556")
        
        # SUB: Receive sensors from ROS1 (Local)
        self.zmq_sub = self.ctx.socket(zmq.SUB)
        self.zmq_sub.connect("tcp://127.0.0.1:5555")
        self.zmq_sub.setsockopt_string(zmq.SUBSCRIBE, "")

        # --- ROS 2 INTERFACE ---
        
        # 1. Subscriber: Listen for /{hostname}/cmd_vel -> Send to ZMQ
        # We subscribe to the NAMESPACED topic only.
        self.create_subscription(
            Twist, 
            f'/{HOSTNAME}/cmd_vel', 
            self.cmd_callback, 
            10
        )
        
        # 2. Publisher: Receive IMU from ZMQ -> Publish to /{hostname}/imu
        self.imu_pub = self.create_publisher(Imu, f'/{HOSTNAME}/imu', 10)

        # Timer to check ZMQ
        self.create_timer(0.01, self.zmq_poll)

    def cmd_callback(self, msg):
        """
        Takes a namespaced command (e.g. from /limo_01/cmd_vel),
        strips the metadata, and sends raw values to ROS1.
        """
        payload = {
            "type": "cmd_vel",
            "linear_x": msg.linear.x,
            "angular_z": msg.angular.z
        }
        self.zmq_pub.send_string(json.dumps(payload))

    def zmq_poll(self):
        """
        Checks if ROS1 sent us any sensor data.
        If so, wraps it in the /limo_01/ namespace and publishes to WiFi.
        """
        try:
            msg_str = self.zmq_sub.recv_string(flags=zmq.NOBLOCK)
            packet = json.loads(msg_str)
            
            if packet["type"] == "imu":
                imu_msg = Imu()
                # Standard headers need a frame_id too
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = f"{HOSTNAME}/imu_link"
                
                d = packet["data"]
                imu_msg.orientation.x = d["or_x"]
                imu_msg.orientation.y = d["or_y"]
                imu_msg.orientation.z = d["or_z"]
                imu_msg.orientation.w = d["or_w"]
                imu_msg.angular_velocity.z = d["av_z"]
                imu_msg.linear_acceleration.x = d["la_x"]
                
                self.imu_pub.publish(imu_msg)
                
        except zmq.Again:
            pass # No data
        except Exception as e:
            self.get_logger().warn(f"Bad ZMQ packet: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = Ros2NamespaceBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
