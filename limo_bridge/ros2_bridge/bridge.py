import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Message Types
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Imu, LaserScan, CompressedImage, CameraInfo
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage  # <-- Direct publishing for namespaced TF

import zmq
import json
import socket
import base64

HOSTNAME = socket.gethostname().replace('-', '_')

class Ros2Bridge(Node):
    def __init__(self):
        super().__init__(f'{HOSTNAME}_bridge')
        
        self.ctx = zmq.Context()
        self.pub = self.ctx.socket(zmq.PUB)
        self.pub.bind("tcp://*:5556") 
        
        self.sub = self.ctx.socket(zmq.SUB)
        self.sub.connect("tcp://127.0.0.1:5555") 
        self.sub.setsockopt_string(zmq.SUBSCRIBE, "")
        
        self.sub.setsockopt(zmq.RCVHWM, 10) # Bumped slightly for TF bursts

        # Configure static TF QoS to latch messages for late subscribers
        static_tf_qos = QoSProfile(
            depth=100,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.ros_pubs = {
            '/tf': self.create_publisher(TFMessage, f'/{HOSTNAME}/tf', 100),
            '/tf_static': self.create_publisher(TFMessage, f'/{HOSTNAME}/tf_static', static_tf_qos),
            
            '/imu': self.create_publisher(Imu, f'/{HOSTNAME}/imu', 10),
            '/odom': self.create_publisher(Odometry, f'/{HOSTNAME}/odom', 10),
            '/scan': self.create_publisher(LaserScan, f'/{HOSTNAME}/scan', QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)),
            
            '/camera/rgb/image_raw/compressed': self.create_publisher(CompressedImage, f'/{HOSTNAME}/camera/rgb/image_raw/compressed', QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)),
            '/camera/depth/image_raw/compressed': self.create_publisher(CompressedImage, f'/{HOSTNAME}/camera/depth/image_raw/compressed', QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)),
            
            '/camera/rgb/camera_info': self.create_publisher(CameraInfo, f'/{HOSTNAME}/camera/rgb/camera_info', 10),
            '/camera/depth/camera_info': self.create_publisher(CameraInfo, f'/{HOSTNAME}/camera/depth/camera_info', 10)
        }

        self.create_subscription(Twist, f'/{HOSTNAME}/cmd_vel', self.cmd_cb, 10)
        self.create_timer(0.01, self.zmq_poll)
        self.get_logger().info(f"Bridge Active for /{HOSTNAME}")

    def cmd_cb(self, msg):
        payload = {
            "topic": "cmd_vel",
            "msg": {
                "linear_x": msg.linear.x,
                "angular_z": msg.angular.z
            }
        }
        self.pub.send_string(json.dumps(payload))

    def format_frame_id(self, frame_id):
        """Helper to prefix TF frames with HOSTNAME. Only 'map' remains global."""
        clean_frame = frame_id.lstrip('/')
        # 'map' stays global; 'odom' and sensor links get namespaced per robot
        if clean_frame == 'map' or clean_frame.startswith(HOSTNAME):
            return clean_frame
        return f"{HOSTNAME}/{clean_frame}"

    def zmq_poll(self):
        while True:
            try:
                msg_str = self.sub.recv_string(flags=zmq.NOBLOCK)
                packet = json.loads(msg_str)
                topic = packet["topic"]
                data = packet["msg"]

                current_time = self.get_clock().now().to_msg()

                if topic in ['/tf', '/tf_static']:
                    tfs = []
                    for tf_data in data["transforms"]:
                        t = TransformStamped()
                        t.header.stamp = current_time
                        
                        t.header.frame_id = self.format_frame_id(tf_data["header"]["frame_id"])
                        t.child_frame_id = self.format_frame_id(tf_data["child_frame_id"])
                        
                        t.transform.translation.x = tf_data["translation"]["x"]
                        t.transform.translation.y = tf_data["translation"]["y"]
                        t.transform.translation.z = tf_data["translation"]["z"]
                        t.transform.rotation.x = tf_data["rotation"]["x"]
                        t.transform.rotation.y = tf_data["rotation"]["y"]
                        t.transform.rotation.z = tf_data["rotation"]["z"]
                        t.transform.rotation.w = tf_data["rotation"]["w"]
                        tfs.append(t)
                        
                    tf_msg = TFMessage()
                    tf_msg.transforms = tfs
                    self.ros_pubs[topic].publish(tf_msg)

                elif topic == '/imu':
                    msg = Imu()
                    msg.header.stamp = current_time
                    msg.header.frame_id = f"{HOSTNAME}/imu_link"
                    msg.orientation.x = data["or"]["x"]
                    msg.orientation.y = data["or"]["y"]
                    msg.orientation.z = data["or"]["z"]
                    msg.orientation.w = data["or"]["w"]
                    msg.angular_velocity.x = data["av"]["x"]
                    msg.angular_velocity.y = data["av"]["y"]
                    msg.angular_velocity.z = data["av"]["z"]
                    msg.linear_acceleration.x = data["la"]["x"]
                    msg.linear_acceleration.y = data["la"]["y"]
                    msg.linear_acceleration.z = data["la"]["z"]
                    self.ros_pubs['/imu'].publish(msg)

                elif topic == '/odom':
                    msg = Odometry()
                    msg.header.stamp = current_time
                    # Updated to ensure the odometry message frame matches the TF tree!
                    msg.header.frame_id = f"{HOSTNAME}/odom"
                    msg.child_frame_id = f"{HOSTNAME}/base_link"
                    msg.pose.pose.position.x = data["pos"]["x"]
                    msg.pose.pose.position.y = data["pos"]["y"]
                    msg.twist.twist.linear.x = data["twist"]["linear"]
                    msg.twist.twist.angular.z = data["twist"]["angular"]
                    self.ros_pubs['/odom'].publish(msg)

                elif topic == '/scan':
                    msg = LaserScan()
                    msg.header.stamp = current_time
                    msg.header.frame_id = f"{HOSTNAME}/laser_link"
                    msg.angle_min = data["angle_min"]
                    msg.angle_increment = data["angle_increment"]
                    msg.range_min = data["range_min"]
                    msg.range_max = data["range_max"]
                    msg.ranges = data["ranges"]
                    self.ros_pubs['/scan'].publish(msg)

                elif topic in ['/camera/rgb/image_raw/compressed', '/camera/depth/image_raw/compressed']:
                    msg = CompressedImage()
                    msg.header.stamp = current_time
                    msg.header.frame_id = f"{HOSTNAME}/camera_link"
                    msg.format = data["format"]
                    msg.data = base64.b64decode(data["data"])
                    self.ros_pubs[topic].publish(msg)

                elif topic in ['/camera/rgb/camera_info', '/camera/depth/camera_info']:
                    msg = CameraInfo()
                    msg.header.stamp = current_time
                    msg.header.frame_id = f"{HOSTNAME}/camera_link"
                    
                    msg.height = data["height"]
                    msg.width = data["width"]
                    msg.distortion_model = data["distortion_model"]
                    msg.d = data["D"]
                    msg.k = data["K"]
                    msg.r = data["R"]
                    msg.p = data["P"]
                    msg.binning_x = data["binning_x"]
                    msg.binning_y = data["binning_y"]
                    msg.roi.x_offset = data["roi"]["x_offset"]
                    msg.roi.y_offset = data["roi"]["y_offset"]
                    msg.roi.height = data["roi"]["height"]
                    msg.roi.width = data["roi"]["width"]
                    msg.roi.do_rectify = data["roi"]["do_rectify"]
                    
                    self.ros_pubs[topic].publish(msg)

            except zmq.Again:
                break 
            except KeyError as e:
                pass 
            except Exception as e:
                self.get_logger().warn(f"Error: {e}")
                break

def main():
    rclpy.init()
    rclpy.spin(Ros2Bridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
