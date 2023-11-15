import numpy as np
from collections import deque
import time
from avoid_col_rotation2 import ht2deq
from publish_position import Triangle_r_v
import matplotlib.pyplot as plt

robx = 3
roby = 3
x = 0
y = 1
vx = 1
vy = 2

dest = Triangle_r_v.dest(x+vx*1 - robx, y+vy - roby, vx, vy)

fig = plt.figure(figsize=(12, 8))
plt.plot(robx, roby, marker=".", markersize=20)
plt.quiver(robx, roby, dest[0], dest[1], angles='xy', scale_units='xy', scale=1, width=0.01)
plt.plot(x, y, marker=".", markersize=20)
plt.plot(dest[0]+robx, dest[1]+roby, marker=".", markersize=20)
plt.show()