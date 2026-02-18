#!/usr/bin/env python
import rospy
import zmq
import json
import yaml # Use YAML or simple dict for config

# Message Types
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int32

# Configuration: Topic Name -> Message Class
TOPIC_MAP = {
    '/imu': Imu,
    '/odom': Odometry,
    '/scan': LaserScan,
    '/limo_status': String
}

class Ros1Bridge:
    def __init__(self):
        rospy.init_node('ros1_bridge', anonymous=True)

        # ZMQ Setup
        self.ctx = zmq.Context()
        self.pub = self.ctx.socket(zmq.PUB)
        self.pub.bind("tcp://*:5555")
        
        self.sub = self.ctx.socket(zmq.SUB)
        self.sub.connect("tcp://127.0.0.1:5556")
        self.sub.setsockopt(zmq.SUBSCRIBE, "") # Py2 syntax
        
        self.poller = zmq.Poller()
        self.poller.register(self.sub, zmq.POLLIN)

        # 1. Setup Subscribers (Robot -> Laptop)
        self.subs = []
        for topic, msg_type in TOPIC_MAP.items():
            # We use a closure (default arg) to capture the topic name correctly
            cb = lambda msg, t=topic: self.generic_callback(msg, t)
            self.subs.append(rospy.Subscriber(topic, msg_type, cb))

        # 2. Setup Publisher (Laptop -> Robot)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        rospy.loginfo("Bridge Initialized for: {}".format(TOPIC_MAP.keys()))

    def generic_callback(self, msg, topic_name):
        """
        Generic callback that serializes any ROS message to a dict
        and sends it over ZMQ.
        """
        try:
            # Helper to convert ROS msg to dict (simplified)
            # For complex msgs (like Scan), we pick key fields to save bandwidth
            data = {}
            
            if topic_name == '/imu':
                data = {
                    "or": {"x": msg.orientation.x, "y": msg.orientation.y, "z": msg.orientation.z, "w": msg.orientation.w},
                    "av": {"x": msg.angular_velocity.x, "y": msg.angular_velocity.y, "z": msg.angular_velocity.z},
                    "la": {"x": msg.linear_acceleration.x, "y": msg.linear_acceleration.y, "z": msg.linear_acceleration.z}
                }
            elif topic_name == '/odom':
                data = {
                    "pos": {"x": msg.pose.pose.position.x, "y": msg.pose.pose.position.y},
                    "twist": {"linear": msg.twist.twist.linear.x, "angular": msg.twist.twist.angular.z}
                }
            elif topic_name == '/scan':
                # Optimization: Downsample scan or just send ranges
                data = {
                    "ranges": list(msg.ranges), # Convert tuple to list
                    "angle_min": msg.angle_min,
                    "angle_increment": msg.angle_increment
                }
            elif topic_name == '/limo_status':
                data = {"data": msg.data}

            payload = {
                "topic": topic_name,
                "msg": data
            }
            self.pub.send_json(payload)
            
        except Exception as e:
            pass # Prevent spamming logs on serialization errors

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            socks = dict(self.poller.poll(0))
            if self.sub in socks:
                try:
                    packet = self.sub.recv_json()
                    if packet.get("topic") == "cmd_vel":
                        # Deserialize Twist
                        t = Twist()
                        t.linear.x = packet["msg"]["linear_x"]
                        t.angular.z = packet["msg"]["angular_z"]
                        self.cmd_pub.publish(t)
                except Exception as e:
                    rospy.logerr("ZMQ Recv Error: {}".format(e))
            rate.sleep()

if __name__ == "__main__":
    Ros1Bridge().run()
