#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 
############################################################

import math
import time
import numpy as np
import random
import rospy

from collections import deque
from collections import defaultdict as ddict

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import patches

from geometry_msgs.msg import PoseArray
from layer2.msg import HTEntityList

class htreceiver():
    def __init__(self) -> None:
        rospy.Subscriber("/human_tracked_l2", HTEntityList, self._in_callback_ht, queue_size=1)
        self.md = movement_decider()
        self.start = time.time()
        self.start2 = None
    #toDO
    def _in_callback_ht(self, msg:HTEntityList):
        now = msg.header.stamp.nsecs * (10 ** (-9)) + msg.header.stamp.secs
        rawlist = msg.list
        htlist = []
        for raw in rawlist:
            htlist.append((raw.id, raw.unique_id, raw.x, raw.y))
        self.md.htreceive_wrapper((now, htlist))

        

    def run(self):
        rospy.loginfo("Start receiver")
        rospy.spin()
    
    def stop(self):
        exit(-1)

class movement_decider():
    robotV = 1
    robotW = 2
    angle_num = 30
    min_t = 1/10
    max_t = 30

    def __init__(self) -> None:
        self.ht2l = ht2list()
        self.angles = [i * math.pi * 2 / movement_decider.angle_num for i in range(movement_decider.angle_num)]
        self.robxya = (0, 0, 0)
        self.targetid = -1
        self.id2index = ddict(int)
    
    def htreceive_wrapper(self, htoutput):
        """
        indexが想像していたものでなかったため、変換するラッパークラス
        """
        t, htlist = htoutput
        for id, unique, x, y in htlist:
            if id in self.id2index.keys():
                self.ht2l.singleht2list((t, self.id2index[id], unique, x, y))
            else:
                index = self.ht2l.unoccupied_num(t)
                print(index)
                self.id2index[id] = index
                self.ht2l.singleht2list((t, index, unique, x, y))
        self.htreveived(t)

    def htreveived(self, t):
        prediction_pos = self.ht2l.prediction_move(t, movement_decider.min_t, movement_decider.max_t)
        humanvxys = self.ht2l.cal_vel(t)
        return self.can_meet(prediction_pos, humanvxys)

    def robreceive(self, roboutput):
        self.robxya = roboutput

    def targetreceive(self, targetid):
        self.targetid = targetid
    
    def targetreset(self):
        self.targetid = -1

    #ここから下未テスト
    def can_meet(self, pos_lists, humanvxys):
        """
        それぞれの人に追いつけるかどうかを探す
        計算量はコマ数*アングル数*人の数
        """
        robotV = movement_decider.robotV
        robotW = movement_decider.robotW
        robxy = self.robxya[0:2]
        nowangle = self.robxya[2]
        min_t = movement_decider.min_t
        max_t = movement_decider.max_t
        targetid = self.targetid
        angles = self.angles

        ans = []
        list_to_avoid = self.to_avoid(robotV, robotW, robxy, pos_lists, humanvxys, angles, nowangle, min_t, max_t)
        for i in range(len(pos_lists)):
            lis, b = self.avoid_meet(robotV, robotW, robxy, pos_lists, humanvxys, angles, nowangle, min_t, max_t, i, list_to_avoid)
            if i == targetid:
                ans2 = (lis, b)

            if len(lis) != 0:
                ans.append(True)
            else:
                ans.append(False)
        print(ans)
        if targetid == -1:
            return ans, None
        else:
            return ans, ans2
            
    def to_avoid(self, robotV, robotW, robxy, pos_lists, humanvxys, angles, nowangle, min_t, max_t):
        """
        ans[human_index][angle] = True or False. Trueのとき衝突しない。\n
        計算量はコマ数*アングル数 
        """
        ans = []
        for i in range(len(pos_lists)):
            tem_ans = ddict(bool)
            for angle in angles:
                tem_ans[angle] = (self.angle_Simulation_rotate(robotV, robotW, robxy, pos_lists[i], humanvxys[i], angle, nowangle, min_t, max_t))
            ans.append(tem_ans)
        return ans

    def avoid_meet(self, robotV, robotW, robxy, pos_lists, humanvxys, angles, nowangle, min_t, max_t, targetid, list_to_avoid):
        """
        targetid にむかって走る角度とすでに会っているかどうかを返す
        計算量はコマ数*アングル数
        """
        
        candidates, is_meet = self.simulation_rotate_meet(robotV, robotW, robxy, pos_lists[targetid], humanvxys[targetid], angles, nowangle, min_t, max_t)
        ans = []
        for _, angle in candidates:
            for i in range(len(pos_lists)):
                if i == targetid:
                    continue

                if not list_to_avoid[i][angle]:
                    break
            else:
                ans.append(angle)
        return ans, is_meet

    def angle_Simulation_rotate_meet(self, robotV, robotW, robxy, pos_list, humanvxy, angle, nowangle, min_t, max_t):
        """
        衝突せずに会えるTrue、衝突するFalse、衝突しないけど会えないFalse\n
        合う時間をt\n
        計算量はコマ数（max_t // min_t)
        """
        if pos_list[0][0] == 100:
            return False, 0
        
        robX, robY = robxy
        humanvx, humanvy = humanvxy
        humanV = (humanvx ** 2 + humanvy ** 2) ** (1/2)
        if angle > math.pi:
            angle -= 2 * math.pi
        
        for i in range(int(max_t // min_t)):
            t = i * min_t
            posX = pos_list[i][0]
            posY = pos_list[i][1]
            if abs(angle - nowangle) >= robotW * min_t / 2:
                if angle > nowangle:
                    nowangle += robotW * min_t
                else:
                    nowangle -= robotW * min_t
                continue
            else:
                robX += robotV * math.cos(nowangle) * min_t
                robY += robotV * math.sin(nowangle) * min_t
                dis = (posX - robX) ** 2 + (posY - robY) ** 2

            if dis > ht2list.human_size ** 2:
                try:
                    targetX = pos_list[i+ht2list.meet_t_index][0]
                    targetY = pos_list[i+ht2list.meet_t_index][1]
                except:
                    break
                dis0 = self.hanchoku(humanvx, humanvy, robX-targetX, robY-targetY)
                if dis0 < (ht2list.human_size/2) ** 2:
                    return True, (t + ((targetX-robX)**2 + (targetY-robY) ** 2) **(1/2) / (humanV))
            else:
                return False, 0
        
        return False, 0
    
    def angle_Simulation_rotate(self, robotV, robotW, robxy, pos_list:list, humanvxy, angle, nowangle, min_t, max_t):
        """
        シミュレーションしてやっていく、、衝突しない場合True\n
        回転を考慮する
        """
        if pos_list[0][0] == 100:
            return True
        
        robX, robY = robxy
        humanvx, humanvy = humanvxy
        humanV = (humanvx ** 2 + humanvy ** 2) ** (1/2)

        if angle > math.pi:
            angle -= 2 * math.pi

        if nowangle > math.pi:
            nowangle -= 2 * math.pi

        for i in range(int(max_t // min_t)):
            t = i * min_t
            posX, posY = pos_list[i]
            if abs(angle - nowangle) >= robotW * min_t / 2:
                if angle > nowangle:
                    nowangle += robotW * min_t
                else:
                    nowangle -= robotW * min_t
            else:
                robX += robotV * math.cos(nowangle) * min_t
                robY += robotV * math.sin(nowangle) * min_t
            dis = (posX - robX) ** 2 + (posY - robY) ** 2
            if dis < ht2list.human_size ** 2:
                return False
        return True
    
    def simulation_rotate_meet(self, robotV, robotW, robxy, pos_list, humanvxy, angles, nowangle, min_t, max_t):
        """
        (t, 会える角度), (会っているかどうか）をreturnする\n
        回転を考慮\n
        もうすでにあっている場合は追従するようにする。その他の人は考えない。
        計算量はコマ数*angleの数
        """
        x, y = pos_list[0]
        humanvx, humanvy = humanvxy
        robx, roby = robxy
        #もうすでにあっているかどうかの判定
        if  (robx - (x + humanvx * ht2list.meet_t)) ** 2 + (roby - (y + humanvy * ht2list.meet_t)) ** 2 <= (ht2list.human_size) ** 2:
            return [(0, (math.atan2(humanvy, humanvx) - math.pi)%(math.pi * 2))], True
        #半直線上にいるけど、あっていない場合
        elif self.hanchoku(humanvx, humanvy, -(x + humanvx * ht2list.meet_t), -(y + humanvy * ht2list.meet_t)) <= (ht2list.human_size/2) ** 2:
            return [(0, (math.atan2(humanvy, humanvx) - math.pi)%(math.pi * 2))], False
        
        #いずれでもない場合
        ans = []
        for angle in angles:
            bl, t = self.angle_Simulation_rotate_meet(robotV, robotW, robxy, pos_list, humanvxy, angle, nowangle, min_t, max_t)
            if bl:
                ans.append((t, angle))
        ans.sort(key=lambda x: x[0])
        return ans, False

    def hanchoku(self, vx, vy, cx, cy):
        if vx * cx + vy * cy < 0:
            return cx ** 2 + cy ** 2
        else:
            try:
                return (cx ** 2 + cy ** 2) - ((cx * vx + cy * vy)**2) / (vx ** 2 + vy ** 2)
            except:
                return cx ** 2 + cy ** 2
    
class ht2list():
    max_list_len = 50
    human_size = 0.1
    meet_t = 0.5
    #min_t * meet_t_index = meet_t
    meet_t_index = 15
    min_t = 1/10
    max_t = 30


    def __init__(self) -> None:
        self.indexdict = ddict(int)
        self.htlist = [ht2deq(None) for _ in range(ht2list.max_list_len)]
        
    def ht2list(self, htoutput):
        """
        htoutput = (t, htlist)
        htlist = [(id, unique, x, y), (),,,]
        """
        t, htlist = htoutput
        for id, uniqueid, x, y in htlist:
            if self.htlist[id].isSameHuman((t, x, y)):
                self.htlist[id].add((t, x, y))
            else:
                self.htlist[id].clear(uniqueid)
                self.htlist[id].add((t, x, y))

    def singleht2list(self, htoutput):
        t, id, uniqueid, x, y = htoutput
        if self.htlist[id].isSameHuman((t, x, y)):
            self.htlist[id].add((t, x, y))
        else:
            self.htlist[id].clear(uniqueid)
            self.htlist[id].add((t, x, y))

    def unoccupied_num(self, t):
        for i, htdeq in enumerate(self.htlist):
            if not htdeq.is_occupied(t):
                return i
            
    
    def cal_vel(self, t):
        ans = []
        for htdeq in self.htlist:
            ans.append(htdeq.calvel(t))
        return ans
    
    def show(self):
        for i in range(3):
            print(self.htlist[i].deq)
    
    def prediction_move(self, t, min_t, max_t):
        """
        人の位置（map）を返す
        min_tごとmax_tまで
        """
        initial_pose = []
        L = int(max_t // min_t)
        for htdeq in self.htlist:
            initial_pose.append(htdeq.calpose(t))
        initial_np = np.array(initial_pose)
        ans_np = np.repeat(initial_np, L, axis = 0).reshape(np.shape(initial_np)[0], -1, 2)
        time = [min_t * i for i in range(L)]
        time_np = np.array(time).reshape(1, -1, 1)
        vel = self.cal_vel(t)
        vel_np = np.repeat(np.array(vel), L, axis=0).reshape(ht2list.max_list_len, -1, 2)
        ans_np = ans_np + time_np * vel_np
        return ans_np.tolist()

    

class ht2deq():
    length = 80
    hv_tolerance = 3
    minimum_for_vel = 4
    time_tolerance = 1

    def __init__(self, unique) -> None:
        self.deq = deque([])
        self.unique = unique
        self.beftxy = [0, 0, 0]
        self.aftxy = [0, 0, 0]

    def add(self, txy):
        if len(self.deq) != ht2deq.length:
            self.deq.append(txy)
        else:
            minus = self.deq.popleft()
            self.deq.append(txy)
            for i in range(3):
                self.beftxy[i] -= minus[i]
                self.beftxy[i] += self.deq[ht2deq.length // 2 - 1][i]
                self.aftxy[i] -= self.deq[ht2deq.length // 2 - 1][i]
                self.aftxy[i] += txy[i]
    
    def is_occupied(self, now):
        """
        このリストが専有されているかどうかを返す。専有されているならTrue、されていないのならばFalse
        """
        if len(self.deq) == 0:
            return False
        else:
            most_recent = self.deq[-1][0]
            if now - most_recent > ht2deq.time_tolerance:
                return False
            else:
                return True


    def calpose(self, t):
        if len(self.deq) == 0:
            return [100, 100]
        elif abs(t - self.deq[-1][0]) < 1:
            return self.deq[-1][1:3]
        else:
            return [100, 100]
        
    def calvel(self, t):
        if len(self.deq) == 0:
            return [0, 0]
        elif abs(t - self.deq[-1][0]) >= 1:
            return [0, 0]
        
        if len(self.deq) == ht2deq.length and self.beftxy[0] != 0:
            vel_x = (self.aftxy[1] - self.beftxy[1]) / (self.aftxy[0] - self.beftxy[0])
            vel_y = (self.aftxy[2] - self.beftxy[2]) / (self.aftxy[0] - self.beftxy[0])
        elif len(self.deq) == ht2deq.length:
            for i, txy in enumerate(self.deq):
                if i >= ht2deq.length // 2:
                    for j in range(3):
                        self.aftxy[j] += txy[j]
                else:
                    for j in range(3):
                        self.beftxy[j] += txy[j]
            vel_x = (self.aftxy[1] - self.beftxy[1]) / (self.aftxy[0] - self.beftxy[0])
            vel_y = (self.aftxy[2] - self.beftxy[2]) / (self.aftxy[0] - self.beftxy[0])
        elif len(self.deq) >= 4:
            locaftxy = [0] * 3
            locbeftxy = [0] * 3
            for i, txy in enumerate(self.deq):
                if i >= len(self.deq) // 2:
                    for j in range(3):
                        locaftxy[j] += txy[j]
                else:
                    for j in range(3):
                        locbeftxy[j] += txy[j]
            vel_x = (locaftxy[1] - locbeftxy[1]) / (locaftxy[0] - locbeftxy[0])
            vel_y = (locaftxy[2] - locbeftxy[2]) / (locaftxy[0] - locbeftxy[0])
        else:
            return [0, 0]

        return [vel_x, vel_y]

    def isSameHuman(self, txy):
        if len(self.deq) == 0:
            return False
        else:
            txy0 = self.deq[-1]
            vel_x = (txy[1] - txy0[1]) / (txy[0] - txy0[0])
            vel_y = (txy[2] - txy0[2]) / (txy[0] - txy0[0])
            if (vel_x ** 2 + vel_y ** 2) ** (1/2) > ht2deq.hv_tolerance:
                return False
            if txy[0] - txy0[0] > ht2deq.time_tolerance:
                return False
        return True

    def isSameHumanById(self, uniqueid) -> bool:
        if self.unique == uniqueid:
            return True
        else:
            return False
        
    def clear(self, uniqueid):
        self.deq = deque([])
        self.unique = uniqueid
        self.beftxy = [0, 0, 0]
        self.aftxy = [0, 0, 0]


class test():
    def __init__(self) -> None:
        self.h1p = [1, 2]
        self.h1v = [0.3, 0.7]

        self.h2p = [3, 8]
        self.h2v = [0.5, 0.8]

        self.h3p = [0, 0]
        self.h3v = [1, 0]

    def humanpos(self, t):
        ans = []
        if t < 2:
            x1 = self.h1p[0] + t * self.h1v[0] + random.random() * 0.1
            y1 = self.h1p[1] + t * self.h1v[1] + random.random() * 0.1
            x2 = self.h2p[0] + t * self.h2v[0] + random.random() * 0.1
            y2 = self.h2p[1] + t * self.h2v[1] + random.random() * 0.1
            return (t, [(1, 1, x1, y1), (2, 2, x2, y2)])
        else:
            x1 = self.h3p[0] + t * self.h3v[0] + random.random() * 0.1
            y1 = self.h3p[1] + t * self.h3v[1] + random.random() * 0.1
            x2 = self.h2p[0] + t * self.h2v[0] + random.random() * 0.1
            y2 = self.h2p[1] + t * self.h2v[1] + random.random() * 0.1
            return (t, [(1, 3, x1, y1), (2, 2, x2, y2)])
        
    def test(self):
        hl = ht2list()
        for i in range(30):
            t = i / 10
            htoutput = self.humanpos(t)
            hl.ht2list(htoutput)
            if i % 10 == 9:
                print(hl.cal_vel(t)[0:3])
                start = time.time()
                print(hl.prediction_move(t, 1/30, 10)[1])
                end = time.time()
                print(end - start)

if __name__ == "__main__":
	# Init ros, create and, run the node
	rospy.init_node("make_path")
	ht = htreceiver()
	try:
		ht.run()
	except rospy.ROSInterruptException:
		pass 