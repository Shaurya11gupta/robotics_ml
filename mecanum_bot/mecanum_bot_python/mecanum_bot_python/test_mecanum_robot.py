#!/usr/bin/env python3

import traceback
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

import numpy as np

width = {"fr": 0.275, "fl": 0.275, "rr": 0.275, "rl": 0.275}
length = {"fr": 0.575, "fl": 0.575, "rr": 0.575, "rl": 0.575}

fr_pub = None
fl_pub = None
rr_pub = None
rl_pub = None

def cmdVelCB(data):
    global fr_pub, fl_pub, rr_pub, rl_pub

    mat = np.matrix([[1, 1, (width["fr"] + length["fr"])],
                     [1, -1, -(width["fl"] + length["fl"])],
                     [1, -1, (width["rr"] + length["rr"])],
                     [1, 1, -(width["rl"] + length["rl"])]])  

    cmd_vel = np.matrix([data.linear.x, data.linear.y, data.angular.z])

    wheel_vel = (np.dot(mat, cmd_vel.T).A1).tolist()

    wv = Float64()

    wv.data = wheel_vel[0]
    fr_pub.publish(wv)

    wv.data = wheel_vel[1]
    fl_pub.publish(wv)

    wv.data = wheel_vel[2]
    rr_pub.publish(wv)

    wv.data = wheel_vel[3]
    rl_pub.publish(wv)

def process():
    global fr_pub, fl_pub, rr_pub, rl_pub

    rclpy.init()

    node = rclpy.create_node('test_mecanum_robot')

    fr_pub = node.create_publisher(Float64, '/front_right_controller/command', 10)
    fl_pub = node.create_publisher(Float64, '/front_left_controller/command', 10)
    rr_pub = node.create_publisher(Float64, '/rear_right_controller/command', 10)
    rl_pub = node.create_publisher(Float64, '/rear_left_controller/command', 10)

    mouse_sub = node.create_subscription(Twist, '/cmd_vel', cmdVelCB, 10)

    while rclpy.ok():
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

try:
  process()
except Exception:
   print(traceback.format_exc())
