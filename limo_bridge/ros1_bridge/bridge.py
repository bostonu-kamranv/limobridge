#!/usr/bin/env python3
import rospy
import zmq
import json
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

class Ros1LocalBridge:
    def __init__(self):
        rospy.init_node('ros1_bridge', anonymous=True)

        # --- ZMQ SETUP ---
        self.ctx = zmq.Context()
        
        # PUB: Send sensor data to ROS2
        self.zmq_pub = self.ctx.socket(zmq.PUB)
        self.zmq_pub.bind("tcp://*:5555")
        
        # SUB: Receive commands from ROS2
        self.zmq_sub = self.ctx.socket(zmq.SUB)
        self.zmq_sub.connect("tcp://127.0.0.1:5556")
        self.zmq_sub.setsockopt_string(zmq.SUBSCRIBE, "")
        
        # Poller for non-blocking checks
        self.poller = zmq.Poller()
        self.poller.register(self.zmq_sub, zmq.POLLIN)

        # --- ROS 1 INTERFACE ---
        # 1. Listen to Hardware (IMU) -> Send to ZMQ
        rospy.Subscriber("/imu", Imu, self.imu_callback)
        
        # 2. Listen to ZMQ -> Publish to Hardware (Motor Driver)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def imu_callback(self, msg):
        # Serialize IMU data (Simplified)
        payload = {
            "type": "imu",
            "data": {
                "or_x": msg.orientation.x,
                "or_y": msg.orientation.y,
                "or_z": msg.orientation.z,
                "or_w": msg.orientation.w,
                "av_z": msg.angular_velocity.z,
                "la_x": msg.linear_acceleration.x
            }
        }
        self.zmq_pub.send_string(json.dumps(payload))

    def run(self):
        rate = rospy.Rate(50) # 50Hz Loop
        while not rospy.is_shutdown():
            # Poll ZMQ for incoming commands
            socks = dict(self.poller.poll(0))
            if self.zmq_sub in socks:
                try:
                    msg_str = self.zmq_sub.recv_string()
                    packet = json.loads(msg_str)
                    
                    if packet["type"] == "cmd_vel":
                        t = Twist()
                        t.linear.x = packet["linear_x"]
                        t.angular.z = packet["angular_z"]
                        self.cmd_pub.publish(t)
                except Exception as e:
                    rospy.logerr(f"Bridge Parse Error: {e}")
            rate.sleep()

if __name__ == "__main__":
    Ros1LocalBridge().run()
