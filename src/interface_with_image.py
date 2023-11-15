#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 
############################################################

import time
import rospy
import tf2_ros
import tf2_geometry_msgs
import tf

from geometry_msgs.msg import PointStamped
from layer2.msg import HTEntityList
from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel
from std_msgs.msg import String

class interface():
    
    def __init__(self) -> None:
        self.humanpoint = []
        self.tfBuffer = tf2_ros.Buffer()
        self.target = -1
        rospy.Subscriber("/usb_cam/image_raw/screenpoint", PointStamped, self._in_callback_sp)
        rospy.Subscriber("/human_tracked_l2", HTEntityList, self._in_callback_ht, queue_size=1)
        rospy.Subscriber('/usb_cam/camera_info', CameraInfo, self._camera_info_cb, queue_size=1)
        rospy.Subscriber('/missing', String, self._in_callback_mis, queue_size=1)

        self.publisher = rospy.Publisher("/target_id_raw", String, queue_size=1)
        self.publish()

    def _in_callback_mis(self, msg:String):
        self.target = -1

    def _in_callback_sp(self, msg:PointStamped):
        pass

    def publish(self):
        while True:
            print(self.target)
            self.publisher.publish(str(self.target))
            time.sleep(1)

    def _in_callback_ht(self, msg:HTEntityList):
        humanpoints_in_cam = []
        rawlist = msg.list
        for raw in rawlist:
            humanpoints_in_cam.append(self.map2cam(raw.x, raw.y, raw.id))
        self.humanpoint = humanpoints_in_cam
        
    def _in_callback_sp(self, msg:PointStamped):
        x, y = msg.point.x, msg.point.y
        if y == 0 or y == 360:
            self.target = -1
            return
        
        min = float("inf")
        for hp in self.humanpoint:
            if abs(x - hp[0]) < min:
                min = abs(x - hp[0])
                id = hp[2]
        self.target = id
    

    def map2cam(self, x, y, id):
        listener = tf.TransformListener()
        ps = PointStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = rospy.Time()
        ps.point.x = x
        ps.point.y = y
        ps.point.z = 0           
        try:
            listener.waitForTransform("head_camera", "map", rospy.Time(), rospy.Duration(10))
            #trans = self.tfBuffer.lookup_transform('head_camera',  "map", rospy.Time())
            transformed_point = listener.transformPoint( "head_camera", ps)
            u, v = self.camera_model.project3dToPixel(
                (transformed_point.point.x,
                transformed_point.point.y,
                transformed_point.point.z)
            )                
            return u, v, id
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)


    def _camera_info_cb(self, msg):
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(msg)

    def run(self):
        rospy.spin()
    

if __name__ == "__main__":
    rospy.init_node("interface")
    interf = interface()
    try:
        interf.run()
    except:
        print("error interface")
