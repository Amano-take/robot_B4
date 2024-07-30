#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 
############################################################


import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import patches

from avoid_col_rotation2 import ht2deq
from avoid_col_rotation2 import ht2list
import rospy
import time
import tf
import tf2_ros, tf2_geometry_msgs
import json
from collections import defaultdict as ddict

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Twist, PointStamped, PoseStamped
from layer2.msg import HTEntityList
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String

class pubposition():
    def __init__(self) -> None:
        rospy.Subscriber("/human_tracked_l2", HTEntityList, self._in_callback_ht, queue_size=1)
        rospy.Subscriber("/target_id_raw", String, self._in_callback_target, queue_size=2)
        self._pub = rospy.Publisher("//move_base_simple/goal", PoseStamped, queue_size=1)
        self._pub_debug = rospy.Publisher("/debug", PointStamped, queue_size=1)
        self._pub_debug2 = rospy.Publisher("/debug2", PointStamped, queue_size=1)
        self._pub_missing_notification = rospy.Publisher("/missing", String, queue_size=2)
        self._pub_dict = rospy.Publisher("/can_meet_dict", String, queue_size=1)
        self.md = set_goal()
        self.start = time.time()
        self.tf2buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf2buffer)


    def _in_callback_ht(self, msg:HTEntityList):

        self.now = msg.header.stamp.nsecs * (10 ** (-9)) + msg.header.stamp.secs
        rawlist = msg.list
        htlist = []
        for raw in rawlist:
            htlist.append((raw.id, raw.unique_id, raw.x, raw.y))
        robpose = self.robotpostion()
        robxy = (robpose.pose.position.x, robpose.pose.position.y)
        ret = self.md.htreceive((self.now, htlist), robxy)
        str_ret = json.dumps(ret)
        self._pub_dict.publish(str_ret)

    def _in_callback_target(self, msg:String):
        """
        HTのidを採用: リストのインデックスではないことに注意
        """
        try:
            targetid = int(msg.data)
            self.md.targetreceive(targetid)
        except Exception as e:
            print("Missing", e)
            self._pub_missing_notification.publish(str("Missing"))
            self._pub.publish(self.robotpostion())
            return
        if targetid == -1:
            return
        robpose = self.robotpostion()
        robxy = (robpose.pose.position.x, robpose.pose.position.y)
        try:   
            dest = self.md.go_target(self.now, pub = self._pub_debug, robxy=robxy, pub2=self._pub_debug2)
            x, y, a = dest
            self._pub.publish(self.xya2ps(x, y, a))
        except Exception as e:
            self._pub_missing_notification.publish(str("Missing"))
            self._pub.publish(self.robotpostion())

                

    def xya2ps(self, x, y, a):
         ps = PoseStamped()
         ps.header.frame_id = "map"
         ps.header.stamp = rospy.Time()
         ps.pose.position.x = x
         ps.pose.position.y = y
         ps.pose.position.z = 0
         ps.pose.orientation.z = math.sin(a/2)
         ps.pose.orientation.w = math.cos(a/2)

         return ps

    def robotpostion(self):
        
        ps = PoseStamped()
        ps.header.frame_id = "base_link"
        ps.header.stamp = rospy.Time()
        ps.pose.position.x = 0
        ps.pose.position.y = 0
        ps.pose.position.z = 0
        ps.pose.orientation.w = 1
        try:
            trans = self.tf2buffer.lookup_transform("map", "base_link", rospy.Time(0))
            tf_position = tf2_geometry_msgs.do_transform_pose(ps, trans)
            return tf_position
        except:
            print("error tf")
        

    def run(self):
        rospy.loginfo("Start receiver")
        rospy.spin()
    
    def stop(self):
        exit(-1)


class set_goal():
    meet_t = 1.2
    distance = 1.2

    def __init__(self) -> None:
        self.ht2l = ht2list()
        self.robxya = (0, 0, 0)
        self.targetid = -1
        self.targetid_raw = -1
        self.id2index = ddict(int)
        self.now = 0
        self.unique = 0
        self.target_uid = -1

    def htreceive(self, htoutput, robxy):
        t, htlist = htoutput
        self.now = t
        ans = []
        for id, unique, x, y in htlist:
            if id in self.id2index.keys():
                self.unique = self.ht2l.singleht2list((t, self.id2index[id], unique, x, y), self.unique)
            else:
                index = self.ht2l.unoccupied_num(t)
                #print(index)
                self.id2index[id] = index
                self.unique = self.ht2l.singleht2list((t, index, unique, x, y), self.unique)
            temp = self.calculate(id, robxy)
            ans.append([x, y, id, temp[0], temp[1]])
        return ans
            

    def targetreceive(self, targetid):
        if targetid == -1:
             self.targetid = -1
             self.targetid_raw = -1
             self.target_uid = -1
             #idが同じで違う人を追う問題をどうにかしたかった。けどあんまり必要なさそう。
             """elif (targetid == self.targetid_raw and self.targetid != self.id2index[targetid] and self.targetid != -1):
             self.targetid = -1
             self.targetid_raw = -1
             self.target_uid = -1
             raise ValueError("Missing")"""
        elif self.target_uid == -1:
             self.target_uid = self.ht2l.htlist[self.id2index[targetid]].unique
             self.targetid_raw = targetid
             self.targetid = self.id2index[targetid]
        else:
             self.target_uid = self.ht2l.htlist[self.id2index[targetid]].unique
             self.targetid_raw = targetid
             self.targetid = self.id2index[targetid]
            
        """"elif self.target_uid != self.ht2l.htlist[self.id2index[targetid]].unique:
             self.targetid = -1
             self.targetid_raw = -1
             self.target_uid = -1
             raise ValueError("Missing... same id but not same uid")"""


    def calculate(self, id, robxy):
        index = self.id2index[id]

        try:
            x, y, vx, vy = self.ht2l.pos_and_vel_with_index(index, self.now)
            if vx ** 2 + vy ** 2 < 10 ** (-1):
                return True, None
            robx, roby = robxy
            goalx, goaly = Triangle_r_v.dest(x+vx*set_goal.meet_t - robx, y+vy*set_goal.meet_t- roby, vx, vy)
            return True, math.sqrt(vx**2 + vy**2)
        
        except Exception as e:
            return False, math.sqrt(vx**2 + vy**2)
        
    def xy2position(self, x, y):
        ans = PointStamped()
        ans.header.frame_id = "map"
        ans.header.stamp = rospy.Time()
        ans.point.x = x
        ans.point.y = y
        return ans
    
        
    def go_target(self,t, pub=None, robxy = None, pub2 = None):
        if self.targetid == -1:
             return None
        try:
            x, y, vx, vy = self.ht2l.pos_and_vel_with_index(self.targetid, t)
            """その人はおそらく消えている"""
            if vx == 0 and vy == 0:
                 self.targetid = -1
                 raise ValueError("this human id is no longer valid")
            
            if pub is not None:
                pub.publish(self.xy2position(x, y))
                 
            #print(vx, vy)
            if robxy is None:
                robx, roby, _ = self.robxya
            else:
                robx, roby = robxy

            
            absvec = math.sqrt((robx - x) ** 2 + (roby - y) ** 2)

            
            if vx ** 2 + vy ** 2 < 10 ** (-1):
                 #速度が小さいときは向きを気にせずにpublish
                 goalx = x + (robx - x) * set_goal.distance / absvec
                 goaly = y + (roby - y) * set_goal.distance / absvec
                 pub2.publish(self.xy2position(goalx, goaly))
                 a = math.atan2(y-roby, x- robx)
                 return goalx, goaly, a
                 """elif absvec < set_goal.distance:
                 #cmd_vel 自前発行必要なさそう
                 return vx, vy, None"""
            else:
                 goalx, goaly = Triangle_r_v.dest(x+vx*set_goal.meet_t - robx, y+vy*set_goal.meet_t-roby, vx, vy)
                 a = math.atan2(vy, vx)
                 a = (a + math.pi) % (2*math.pi)
                 return goalx+robx, goaly+roby, a
        except Exception as e:
            print(e)
            return None
            #raise ValueError....
            
             
             



class Triangle_r_v():
    Robot_speed = 0.6
    Near_zero = 0.01

    def __init__(self):
        return 
    
    @staticmethod
    def dest(x, y, vx, vy):
        a = vx ** 2 + vy ** 2 - Triangle_r_v.Robot_speed ** 2
        b = 2 * (x * vx + y * vy)
        c = x ** 2 + y ** 2
        if abs(a) > Triangle_r_v.Near_zero:
            k = Triangle_r_v.solv_quadratic_equation(a, b, c)
        else:
            k = Triangle_r_v.solv_linear_equation(b, c)
        
        if k < 0:
            raise ValueError("can't reach!!")
        dest_x = x + k * vx
        dest_y = y + k * vy

        return dest_x, dest_y
    
    @staticmethod
    def solv_quadratic_equation(a, b, c):
        if (b**2 - 4*a*c) < 0:
            return -10
        
        D = (b**2 - 4*a*c) ** (1/2)
        x_1 = (-b + D) / (2 * a)
        x_2 = (-b - D) / (2 * a)
        
        if x_2 >= 0:
            return x_2
        return x_1
    
    def solv_linear_equation(b, c):
        return - c / b
     
if __name__ == "__main__":
    # Init ros, create and, run the node
    rospy.init_node("make_path")
    ht = pubposition()
    try:
        ht.run()
    except rospy.ROSInterruptException:
        pass 