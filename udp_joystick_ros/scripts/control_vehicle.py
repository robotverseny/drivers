#!/usr/bin/env python

import socket
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
#import autoware_msgs.msg as auwmsg
import numpy as np
from rclpy.qos import QoSProfile
import time
from std_msgs.msg import Float32
import threading

class UdpJoystickServer:
    start_controller_state = np.array([0, 0, 0, 0])

    def __init__(self, port):
        self.port = port
        self.controller_state = self.start_controller_state
        # self.pub_tw = rospy.Publisher("ctrl_cmd", auwmsg.ControlCommandStamped, queue_size=10)
        self.pub_tw = PUBLISHER
        # rospy.loginfo("Publishing ctrl_cmd [autoware_msgs/ControlCommandStamped]")
        print("Publishing cmd_vel [geometry_msgs/msg/Twist]")

    def start_server(self):
        thread = threading.Thread(target = self.recieve_msgs)
        self._stop = threading.Event()
        thread.start()

    def stop_server(self):
        self._stop.set()

    def recieve_msgs(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('',self.port))
        sock.settimeout(1.0)
        s = "2000100014991500"
        s[0:4]
        s[4:8]
        s[8:12]
        s[12:16]
        # msg_aw = auwmsg.ControlCommandStamped()
        # msg_twist = Twist()
        cnt = 1500
        div = 500
        multiply = 1.0
        while(not self._stop.isSet()):
                try:
                    msg, _ = sock.recvfrom(1024)
                    print("Msg: " + str(msg))
                    try:
                        m1 = (float(str(msg)[0:4])   - cnt) / div * multiply
                        m2 = (float(str(msg)[4:8])   - cnt) / div * multiply
                        m3 = (float(str(msg)[8:12])  - cnt) / div * multiply
                        m4 = (float(str(msg)[12:16]) - cnt) / div * multiply
                        self.controller_state = np.array([m1, m2, m3, m4])
                        # msg_aw.cmd.linear_velocity = m4 * -20
                        # msg_aw.cmd.steering_angle = m3 * -0.5
                        # self.pub_tw.publish(msg_aw)
                        # self.pub_tw.publish(msg_twist)
                        #rospy.loginfo(self.controller_state)
                        print(str(self.controller_state))
                    except (SyntaxError,ValueError):
                        # rospy.logerr("Malformed UDP msg to controller server")
                        print("Malformed UDP msg to controller server")
                except socket.timeout:
                    # rospy.logwarn("Didn't receive command before timeout, listening again")
                    print("Didn't receive command before timeout, listening again")
        sock.close()

    def get_controller_state_ref(self):
        return self.controller_state

def cmd_callback(data):
    msg_twist = Twist()
    msg_twist.linear.x = 0
    msg_twist.linear.y = 0
    msg_twist.linear.z = 0
    msg_twist.angular.x = 0
    msg_twist.angular.y = 0
    msg_twist.angular.z = 0
    PUBLISHER.publish(msg_twist)

def main():
    rclpy.init()
    global NODE
    global PUBLISHER
    
    #rospy.init_node("udp_control", disable_signals=True)
    NODE = rclpy.create_node('udp_control')
    qos = QoSProfile(depth=10)
    PUBLISHER = NODE.create_publisher(Twist,'/cmd_vel', qos)
    
    #try:
    #    port = rospy.get_param("udp_control/udp_port")
    #except:
    #    port = 50505
    port = 50505
    
    PUBLISHER = NODE.create_publisher(Twist,'/ctrl_cmd', qos)
    NODE.create_subscription(Twist,'cmd_vel',cmd_callback,qos)
    
    try:
        server = UdpJoystickServer(port)
        # rospy.loginfo("Starting server... Port: " + str(port))
        print("Starting server... Port: " + str(port))
        server.start_server()
        controller_ref = server.get_controller_state_ref()
        while True:
            time.sleep(1)
    except ValueError:    
        # rospy.logerr("Invalid port input, exiting...")
        print("Invalid port input, exiting...")
    except KeyboardInterrupt:   
        server.stop_server()
        # rospy.loginfo("Shutting down ros node...")
        print("Shutting down ros node...")

    rclpy.spin(NODE)
    NODE.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

