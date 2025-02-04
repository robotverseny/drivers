#!/usr/bin/env python

import socket
import rospy
import geometry_msgs.msg as geomsg
import autoware_msgs.msg as auwmsg
import numpy as np
import time
import threading

class UdpJoystickServer:
    start_controller_state = np.array([0, 0, 0, 0])

    def __init__(self, port):
        self.port = port
        self.controller_state = self.start_controller_state
        self.pub_tw = rospy.Publisher("vehicle_cmd", auwmsg.VehicleCmd, queue_size=10)
        rospy.loginfo("Publishing vehicle_cmd [autoware_msgs/VehicleCmd]")

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
        #s = "2000100014991500"
        #s[0:4]
        #s[4:8]
        #s[8:12]
        #s[12:16]
        msg_aw = auwmsg.VehicleCmd()
        cnt = 1500
        div = 500
        multiply = 1.0
        while(not self._stop.isSet()):
                try:
                    msg, _ = sock.recvfrom(1024)
                    #print("Msg: " + str(msg))
                    try:
                        m1 = (float(str(msg)[0:4])   - cnt) / div * multiply
                        m2 = (float(str(msg)[4:8])   - cnt) / div * multiply
                        m3 = (float(str(msg)[8:12])  - cnt) / div * multiply
                        m4 = (float(str(msg)[12:16]) - cnt) / div * multiply
                        self.controller_state = np.array([m1, m2, m3, m4])
                        msg_aw.header.stamp = rospy.Time.now()
                        msg_aw.twist_cmd.twist.linear.x = m4
                        msg_aw.twist_cmd.twist.angular.z = m3
                        self.pub_tw.publish(msg_aw)
                        #rospy.loginfo(self.controller_state)
                    except (SyntaxError,ValueError):
                        rospy.logerr("Malformed UDP msg to controller server")
                except socket.timeout:
                    rospy.logwarn("Didn't receive command before timeout, listening again")
        sock.close()

    def get_controller_state_ref(self):
        return self.controller_state

if __name__ == "__main__":

    rospy.init_node("udp_control", disable_signals=True)
    try:
        port = rospy.get_param("udp_control/udp_port")
    except:
        port = 50505
    try:
        server = UdpJoystickServer(port)
        rospy.loginfo("Starting server... Port: " + str(port))
        server.start_server()
        #controller_ref = server.get_controller_state_ref()
        while True:
            time.sleep(1)
    except ValueError:    
        rospy.logerr("Invalid port input, exiting...")
    except KeyboardInterrupt:   
        server.stop_server()
        rospy.loginfo("Shutting down ros node...")
