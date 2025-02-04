#!/usr/bin/env python

# a test script, which publishes /joy topic, not a final version

import rospy
import sensor_msgs.msg as sensmsg
import random

i = 0
rospy.init_node("tmp_pub", anonymous = True)
pub_joy = rospy.Publisher("joy", sensmsg.Joy, queue_size=10)
rospy.logwarn("ONLY TEST PURPOSE")
rate = rospy.Rate(20)
sm = sensmsg.Joy()
sm.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
while not rospy.is_shutdown():
    i += 1
    if i > 30:
        r = random.uniform(0.4, 0.6)
        sm.axes = [0.0, r, 0.0]
    else:
        r = random.uniform(0.0, 0.03)
        sm.axes = [0.0, 0.1, 0.0]
    if i > 60:
        i = 0
    pub_joy.publish(sm)
    rate.sleep()