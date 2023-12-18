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
from cv_bridge import CvBridge, CvBridgeError


class interface():
    publish_frequency = 1
    frame_size_x = 640
    frame_size_y = 480
    def __init__(self) -> None:
        self.humanpoint = []
        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)
        self.target = -1
        self.bridge = CvBridge()
        
        rospy.Subscriber("/my_image_raw/screenpoint", PointStamped, self._in_callback_sp)
        rospy.Subscriber("/can_meet_dict", String, self._in_callback_ht, queue_size=1)
        rospy.Subscriber('/usb_cam/camera_info', CameraInfo, self._camera_info_cb, queue_size=1)
        rospy.Subscriber('/missing', String, self._in_callback_mis, queue_size=1)

        self.publisher = rospy.Publisher("/target_id_raw", String, queue_size=1)
        self.pubimage = rospy.Publisher("/my_image_raw", Image, queue_size=1)
        rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self._in_callback_image)
        self.stop = False
        self.get_screenpoint_time = time.time() - 1
        #self.publish()
        
    def _in_callback_image(self, msg:Image):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            input_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)
   
        try:
            cv_array = cv2.resize(input_image, (640, 480))
            s = 100
            r = 100
            for hp1, hp2 in self.humanpoint:
                x1, y1, id, flag = hp1
                x2, y2, id, flag = hp2
                x1, x2 = min(x1, x2), max(x1, x2)
                if x2 - x1 < 20:
                    a = 20 - (x2 - x1)
                    x2 += a / 2
                    x1 += a / 2
                y1, y2 = min(y1, y2), max(y1, y2)
                if flag:
                        color = (0, 255, 0)
                else :
                    color = (0, 0, 255)

                if x1 >= interface.frame_size_x:
                    p1 = [interface.frame_size_x - 10, s]
                    p2 = [interface.frame_size_x, s + 5]
                    p3 = [interface.frame_size_x - 10, s + 10]
                    s += 20
                    cv_array = cv2.fillConvexPoly(cv_array, np.array([p1, p2, p3]), color, lineType=cv2.LINE_AA)
                elif x2 <= 0:
                    p1 = [10, r]
                    p2 = [0, r + 5]
                    p3 = [10, r + 10]
                    r += 20
                    cv_array = cv2.fillConvexPoly(cv_array, np.array([p1, p2, p3]), color, lineType=cv2.LINE_AA)
                else:
                    #TODO
                    if self.target == id:
                        thick = 10
                    else:
                        thick = 1
                    
                    cv_array = cv2.rectangle(cv_array, (int(x1), int(y1)), (int(x2), int(y2)), color, thick,  lineType=cv2.LINE_AA)
            self.pubimage.publish(self.bridge.cv2_to_imgmsg(cv_array, encoding="bgr8"))
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
        rawlist = json.loads(msg.data)
        #rospy.loginfo(rawlist)
        humanpoints_in_cam = self.map2cam(rawlist)
        if humanpoints_in_cam is not None:
            self.humanpoint = humanpoints_in_cam
            print(time.time())

        
    def _in_callback_sp(self, msg:PointStamped):
        t = time.time()
        if t - self.get_screenpoint_time < 0.5:
            self.target = -1
            return 
        
        self.get_screenpoint_time = t
        x, y = msg.point.x, msg.point.y
        if y == 0 or y == 360 or x == 0 or x == interface.frame_size_x:
            self.target = -1
            return
        
        min = float("inf")
        if len(self.humanpoint) == 0 :
            self.target = -1
            return
        
        for hp in self.humanpoint:
            if abs(x - hp[0][0]) < min:
                min = abs(x - hp[0][0])
                id = hp[0][2]
        self.target = id
    

    def map2cam(self, li):
        """
        [x, y, id, flag], [], ...
        """
        #listener = tf.TransformListener()
                  
        try:
            #listener.waitForTransform("head_camera", "map", rospy.Time(), rospy.Duration(10))
            trans = self.tfBuffer.lookup_transform('head_camera',  "map", rospy.Time(0))
            ans = []
            for raw in li:
                temp = []
                x, y, id, flag = raw
                ps = PointStamped()
                ps.header.frame_id = "map"
                ps.header.stamp = rospy.Time()
                ps.point.x = x -0.1
                ps.point.y = y - 0.1
                ps.point.z = 1.7
                #transformed_point = listener.transformPoint( "head_camera", ps)
                transformed_point = tf2_geometry_msgs.do_transform_point(ps, trans)
                if transformed_point.point.z < 0:
                    continue
                u, v = self.camera_model.project3dToPixel(
                    (transformed_point.point.x,
                    transformed_point.point.y,
                    transformed_point.point.z)
                ) 
                temp.append([u, v, id, flag])
                ps = PointStamped()
                ps.header.frame_id = "map"
                ps.header.stamp = rospy.Time()
                ps.point.x = x + 0.1
                ps.point.y = y + 0.1
                ps.point.z = 0
                #transformed_point = listener.transformPoint("head_camera", ps)
                transformed_point = tf2_geometry_msgs.do_transform_point(ps, trans)
                if transformed_point.point.z < 0:
                    continue
                u, v = self.camera_model.project3dToPixel(
                    (transformed_point.point.x,
                    transformed_point.point.y,
                    transformed_point.point.z)
                )            
                temp.append([u, v, id, flag])
                ans.append(temp)
            
            return ans
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
