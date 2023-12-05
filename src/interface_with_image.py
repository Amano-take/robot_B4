#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 
############################################################

import random
import time
import rospy
import tf2_ros
import tf2_geometry_msgs
import json
import tf
import cv2
import numpy as np

from geometry_msgs.msg import PointStamped
from layer2.msg import HTEntityList
from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge


class interface():
    publish_frequency = 1
    def __init__(self) -> None:
        self.humanpoint = []
        self.tfBuffer = tf2_ros.Buffer()
        self.target = -1
        self.bridge = CvBridge()
        
        rospy.Subscriber("/my_image_raw/screenpoint", PointStamped, self._in_callback_sp)
        rospy.Subscriber("/can_meet_dict", String, self._in_callback_ht, queue_size=1)
        rospy.Subscriber('/usb_cam/camera_info', CameraInfo, self._camera_info_cb, queue_size=1)
        rospy.Subscriber('/missing', String, self._in_callback_mis, queue_size=1)

        self.publisher = rospy.Publisher("/target_id_raw", String, queue_size=1)
        self.pubimage = rospy.Publisher("/my_image_raw", Image, queue_size=1)
        rospy.Subscriber("/usb_cam/image_raw", Image, self._in_callback_image)
        self.stop = False
        self.get_screenpoint_time = time.time() - 1
        #self.publish()
        
    def _in_callback_image(self, msg:Image):
        try:
            cv_array = self.bridge.imgmsg_to_cv2(msg)
            s = 100
            r = 100
            for hp in self.humanpoint:
                if hp[3]:
                        color = (0, 255, 0)
                else :
                    color = (255, 0, 0)

                if hp[0] >= 640:
                    p1 = [630, s]
                    p2 = [640, s + 5]
                    p3 = [630, s + 10]
                    s += 20
                    cv_array = cv2.fillConvexPoly(cv_array, np.array([p1, p2, p3]), color, lineType=cv2.LINE_AA)
                elif hp[0] <= 0:
                    p1 = [10, r]
                    p2 = [0, r + 5]
                    p3 = [10, r + 10]
                    r += 20
                    cv_array = cv2.fillConvexPoly(cv_array, np.array([p1, p2, p3]), color, lineType=cv2.LINE_AA)
                else:
                    a = (int(hp[0]) - 40, 370)
                    b = (int(hp[0])+ 40, 100)
                    if self.target == hp[2]:
                        thick = 10
                    else:
                        thick = 1

                    
                    cv_array = cv2.rectangle(cv_array, a, b, color, thick,  lineType=cv2.LINE_AA)
            self.pubimage.publish(self.bridge.cv2_to_imgmsg(cv_array, encoding="rgb8"))
        except Exception as err:
            print(err)

    def _in_callback_mis(self, msg:String):
        self.target = -1

    def _in_callback_sp(self, msg:PointStamped):
        pass

    def stop(self):
        print("kokodesuyo")
        self.stop = True

    def publish(self):
        try:
            while True:
                if rospy.core.is_shutdown():
                    break
                self.publisher.publish(str(self.target))
                time.sleep(1 / interface.publish_frequency)
        except KeyboardInterrupt:
            print("kokoha?")
            rospy.core.signal_shutdown('keyboard interrupt')

    def _in_callback_ht(self, msg:String):
        humanpoints_in_cam = []
        rawlist = json.loads(msg.data)
        #rospy.loginfo(rawlist)
        for raw in rawlist:
            humanpoints_in_cam.append(self.map2cam(raw))
        self.humanpoint = humanpoints_in_cam
        
    def _in_callback_sp(self, msg:PointStamped):
        t = time.time()
        if t - self.get_screenpoint_time < 0.5:
            self.target = -1
            return 
        
        self.get_screenpoint_time = t
        x, y = msg.point.x, msg.point.y
        if y == 0 or y == 360 or x == 0 or x == 640:
            self.target = -1
            return
        
        min = float("inf")
        if len(self.humanpoint) == 0 :
            self.target = -1
            return
        
        for hp in self.humanpoint:
            if abs(x - hp[0]) < min:
                min = abs(x - hp[0])
                id = hp[2]
        self.target = id
    

    def map2cam(self, li):
        """
        x, y, id, flag
        """
        x, y, id, flag = li
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
            return u, v, id, flag
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)


    def _camera_info_cb(self, msg):
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(msg)

    def run(self):
        self.publish()
    

if __name__ == "__main__":
    rospy.init_node("interface")
    try:
        interf = interface()
        interf.run()
    except KeyboardInterrupt:
        interf.stop()
