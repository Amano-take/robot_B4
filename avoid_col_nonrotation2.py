import math
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import patches
import bisect
from collections import deque
import heapq
from collections import defaultdict as ddict
import numpy as np
import random

class ht2deq():
    length = 100
    hv_tolerance = 3
    minimum_for_vel = 4

    def __init__(self, unique) -> None:
        self.deq = deque([])
        self.unique = unique
        self.beftxy = None
        self.aftxy = None

    def add(self, *txy):
        if len(self.deq) != ht2deq.length:
            self.deq.append(tuple(txy))
        else:
            minus = self.deq.popleft()
            self.deq.appendleft(tuple(txy))
            for i in range(3):
                self.beftxy[i] -= minus[i]
                self.beftxy[i] += self.deq[ht2deq.length // 2 - 1][i]
                self.aftxy[i] -= self.deq[ht2deq.length // 2 - 1][i]
                self.aftxy[i] += txy[i]
            
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
        
        if len(self.deq) == ht2deq.length and self.beftxy != None:
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
        return True

    def isSameHumanById(self, uniqueid) -> bool:
        if self.unique == uniqueid:
            return True
        else:
            return False
        
    def clear(self, uniqueid):
        self.deq = deque([])
        self.unique = uniqueid
        self.beftxy = None
        self.aftxy = None
        
class ht2list():
    max_list_len = 50
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
            if self.htlist[id].isSameHumanById(uniqueid):
                self.htlist[id].add(t, x, y)
            else:
                self.htlist[id].clear(uniqueid)
                self.htlist[id].add(t, x, y)
    
    def cal_vel(self, t):
        ans = []
        for htdeq in self.htlist:
            ans.append(htdeq.calvel(t))
        return ans
    
    def show(self):
        for i in range(3):
            print(self.htlist[i].deq)
    
    def prediction_move(self, t, min_t, max_t):
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
    
    #toDo




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
ta = test()
ta.test()
    
        