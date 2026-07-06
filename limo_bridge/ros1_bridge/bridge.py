#!/usr/bin/env python
import rospy
import zmq
import json
import math
import base64
import sys

# Python 2/3 compatibility for Queue (Melodic vs Noetic)
if sys.version_info >= (3, 0):
    from queue import Queue, Full, Empty
else:
    from Queue import Queue, Full, Empty

# Message Types
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan, CompressedImage, CameraInfo
from nav_msgs.msg import Odometry

TOPIC_MAP = {
    '/imu': Imu,
    '/odom': Odometry,
    '/scan': LaserScan,
    '/camera/rgb/image_raw/compressed': CompressedImage,
    '/camera/depth/image_raw/compressed': CompressedImage, 
    '/camera/rgb/camera_info': CameraInfo,
    '/camera/depth/camera_info': CameraInfo
}

class Ros1Bridge:
    def __init__(self):
        rospy.init_node('ros1_bridge', anonymous=True)

        self.ctx = zmq.Context()
        self.pub = self.ctx.socket(zmq.PUB)
        self.pub.bind("tcp://*:5555")
        
        self.sub = self.ctx.socket(zmq.SUB)
        self.sub.connect("tcp://127.0.0.1:5556")
        
        # PyZMQ string/byte compatibility
        if sys.version_info >= (3, 0):
            self.sub.setsockopt(zmq.SUBSCRIBE, b"")
        else:
            self.sub.setsockopt(zmq.SUBSCRIBE, "") 
        
        self.poller = zmq.Poller()
        self.poller.register(self.sub, zmq.POLLIN)

        self.send_queue = Queue(maxsize=100)
        
        # --- THE FIX: Track frames to throttle at the source ---
        self.frame_counters = {}

        self.subs = []
        for topic, msg_type in TOPIC_MAP.items():
            cb = lambda msg, t=topic: self.generic_callback(msg, t)
            self.subs.append(rospy.Subscriber(topic, msg_type, cb))

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.loginfo("Bridge Initialized. Listening for: {}".format(TOPIC_MAP.keys()))

    def generic_callback(self, msg, topic_name):
        # --- THROTTLE LOGIC: Drop 9 out of 10 heavy frames BEFORE processing ---
        if topic_name not in self.frame_counters:
            self.frame_counters[topic_name] = 0
            
        heavy_topics = [
            '/scan', 
            '/camera/rgb/image_raw/compressed', 
            '/camera/depth/image_raw/compressed'
        ]

        if topic_name in heavy_topics:
            self.frame_counters[topic_name] += 1
            # If it is not the 10th frame, exit immediately to save CPU
            if self.frame_counters[topic_name] % 10 != 0:
                return 
        # -----------------------------------------------------------------------

        try:
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
                clean_ranges = []
                max_r = msg.range_max
                for r in msg.ranges:
                    if math.isinf(r) or math.isnan(r):
                        clean_ranges.append(max_r)
                    else:
                        clean_ranges.append(round(r, 3))
                data = {
                    "ranges": clean_ranges,
                    "angle_min": msg.angle_min,
                    "angle_increment": msg.angle_increment,
                    "range_min": msg.range_min,
                    "range_max": msg.range_max
                }
            elif topic_name in ['/camera/rgb/image_raw/compressed', '/camera/depth/image_raw/compressed']:
                encoded_data = base64.b64encode(msg.data).decode('ascii')
                data = {
                    "format": msg.format,
                    "data": encoded_data
                }
            elif topic_name in ['/camera/rgb/camera_info', '/camera/depth/camera_info']:
                data = {
                    "height": msg.height,
                    "width": msg.width,
                    "distortion_model": msg.distortion_model,
                    "D": list(msg.D),
                    "K": list(msg.K),
                    "R": list(msg.R),
                    "P": list(msg.P),
                    "binning_x": msg.binning_x,
                    "binning_y": msg.binning_y,
                    "roi": {
                        "x_offset": msg.roi.x_offset,
                        "y_offset": msg.roi.y_offset,
                        "height": msg.roi.height,
                        "width": msg.roi.width,
                        "do_rectify": msg.roi.do_rectify
                    }
                }

            payload = {
                "topic": topic_name,
                "msg": data
            }
            
            try:
                self.send_queue.put_nowait(payload)
            except Full:
                try:
                    self.send_queue.get_nowait() 
                    self.send_queue.put_nowait(payload) 
                except Empty:
                    pass
            
        except Exception as e:
            rospy.logerr_throttle(1, "Serialization Error on {}: {}".format(topic_name, e))

    def run(self):
        rate = rospy.Rate(100) 
        while not rospy.is_shutdown():
            socks = dict(self.poller.poll(0))
            if self.sub in socks:
                try:
                    packet = self.sub.recv_json()
                    if packet.get("topic") == "cmd_vel":
                        t = Twist()
                        t.linear.x = packet["msg"]["linear_x"]
                        t.angular.z = packet["msg"]["angular_z"]
                        self.cmd_pub.publish(t)
                except Exception as e:
                    rospy.logerr("ZMQ Recv Error: {}".format(e))
            
            while not self.send_queue.empty():
                try:
                    payload = self.send_queue.get_nowait()
                    self.pub.send_json(payload)
                except Exception as e:
                    pass

            rate.sleep()

if __name__ == "__main__":
    Ros1Bridge().run()
