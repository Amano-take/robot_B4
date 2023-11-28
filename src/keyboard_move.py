#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 
############################################################



import rospy
from pynput.keyboard import Key, Listener

from geometry_msgs.msg import Twist

pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
def on_press(key):
    print(key)
    
    if key == Key.left:
        vel = Twist()
        vel.angular.z = -10
        pub.publish(vel)
    
    if key == Key.right:
        vel = Twist()
        vel.angular.z = 10
        pub.publish(vel)

    if key == Key.up:
        vel = Twist()
        vel.linear.x = 10
        pub.publish(vel)

    if key == Key.down:
        vel = Twist()
        vel.linear.x = -10
        pub.publish(vel)
        

if __name__ == "__main__":
    rospy.init_node("keyB")
    listener = Listener(on_press=on_press)
    listener.start()
    while not rospy.core.is_shutdown():
        pass