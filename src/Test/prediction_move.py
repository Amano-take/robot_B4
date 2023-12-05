#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from collections import deque
import math
import matplotlib.pyplot as plt
import numpy as np
import rospy
import time

from geometry_msgs.msg import PoseStamped
#from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import MarkerArray
#from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
#
class Triangle_r_v():
	Robot_speed = 2
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
		D = (b**2 - 4*a*c) ** (1/2)
		x_1 = (-b + D) / (2 * a)
		x_2 = (-b - D) / (2 * a)
		if (b**2 - 4*a*c) < 0:
			return -10
		if x_2 >= 0:
			return x_2
		return x_1
	
	def solv_linear_equation(b, c):
		return - c / b


"""
x = -2.5
y = 4
vx = math.cos(-0.2)
vy = math.sin(-0.2)

dx, dy = Triangle_r_v.dest(x, y, vx, vy)
plt.figure(figsize=(10,10))
plt.plot(x, y, marker='.', markersize=30)
plt.plot(0, 0, marker='.', markersize = 30)
plt.text(0, 0.3, "robot", size=70)
plt.text(x, y+0.3, 'human', size=70)
plt.plot(dx, dy, marker='*', markersize=30)
plt.quiver(0, 0, dx, dy, angles='xy', scale_units='xy', scale=2, width=0.01)
plt.quiver(x, y, dx-x, dy-y, angles='xy', scale_units='xy', scale=2, width=0.01)
plt.xlim([-3, 3])
plt.ylim([-0.5, 5.5])
plt.grid()

plt.savefig("./amano_senior_thesis/Img/Prediction1.png")
"""

class Human_velocity():
	#List[i] = [ n (msec), x(map), y(map)]
	Const_n = 50
	Volume_Queue = 2 * Const_n
	tolerance = 0.1
	sec_multiple = 1000

	def __init__(self, value=[0, 0, 0]):
		self.q = deque()
		self.q.append(value.copy())
		self.previous_value = value.copy()
		self.velocity = 0
		pass

	def append_and_pop(self, value):
		value= value.copy()
		#print(self.q)
		if not self.is_same_human(value):
			self.__init__(value)
			print("human-change")
			return 0, 0

		if len(self.q) < Human_velocity.Volume_Queue:
			self.q.append(value)
			return self.cal_vel()
		else:
			self.q.append(value)
			self.q.popleft()
			return self.cal_vel()

	def is_same_human(self, value):
		velocity = self.cal_dis(self.previous_value, value) / (value[0] - self.previous_value[0])
		self.previous_value = value
		return velocity < Human_velocity.tolerance

	
	def cal_dis(self, v1, v2):
		x1 = v1[1]
		y1 = v1[2]
		x2 = v2[1]
		y2 = v2[2]
		return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

	def cal_vel(self):
		#must refine algo
		n = len(self.q) // 2
		if n < 3:
			return 0, 0
		sum_msec1 = 0
		sum_place1_x = 0
		sum_place1_y = 0

		for i in range(n):
			sum_msec1 += self.q[i][0]
			sum_place1_x += self.q[i][1]
			sum_place1_y += self.q[i][2]
		
		msec1 = sum_msec1 / n
		place1_x = sum_place1_x / n
		place1_y = sum_place1_y / n

		sum_msec2 = 0
		sum_place2_x = 0
		sum_place2_y = 0

		for i in range(n, 2*n):
			sum_msec2 += self.q[i][0]
			sum_place2_x += self.q[i][1]
			sum_place2_y += self.q[i][2]
		
		msec2 = sum_msec2 / n
		place2_x = sum_place2_x / n
		place2_y = sum_place2_y / n

		dif_msec = msec2 - msec1
		dif_x = place2_x - place1_x
		dif_y = place2_y - place1_y

		self.velocity = dif_x * Human_velocity.sec_multiple / dif_msec, dif_y* Human_velocity.sec_multiple / dif_msec

		return self.velocity
	


"""
value = [0, 0, 0]
hv = Human_velocity(value)


for i in range(200):
	t = np.random.normal(loc=0, scale=0.3, size=1)
	a = np.random.normal(loc = 0, scale=0.001, size=2)
	value[0] += 13 + t[0]
	value[1] += 0.00123 * 13 + a[0]
	value[2] += 0.00256 * 13 + a[1]
	if i == 98:
		value[1] = 13
		value[2] = 14
	v = hv.append_and_pop(value)
	if i % 10 == 0:
		print(v)
"""

#ToDO!!
class Ht_marker2point(object):
	def __init__(self):
		rospy.Subscriber("/ht_markers", MarkerArray, self._in_callback, queue_size=1)
		self._pub = rospy.Publisher("/amano_move_base_simple/goal", PoseStamped, latch=False, queue_size = 1)
		self.human_velocity_list = []
		#self._pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, latch=False, queue_size = 10)
		#self.point_pub = rospy.Publisher("goal_point", PointStamped, latch=False, queue_size = 10)

	def _in_callback(self, msg:MarkerArray):
		n = len(msg.markers) // 2
		for i in range(n):
			x = msg.markers[i*2].pose.position.x
			y = msg.markers[i*2].pose.position.y
			msec = msg.markers[i*2].header####ToDo
			try:
				self.human_velocity_list[i].append_and_pop([x, y, msec])
			except IndexError:
				self.human_velocity_list.append(Human_velocity([msec, x, y]))

		"""
		point = PoseStamped()
		quaternion:Quaternion = msg.markers[0].pose.orientation 
		theta = math.acos(quaternion.w) * 2
		point.pose.orientation.w = math.cos((theta + math.pi / 2) / 2)
		point.pose.orientation.x = 0
		point.pose.orientation.y = 0
		point.pose.orientation.z = math.sin((theta + math.pi / 2) / 2)
		adder_x =  math.cos(theta - math.pi / 2) 
		adder_y =  math.sin(theta - math.pi / 2) 
		point.pose.position.x = msg.markers[0].pose.position.x + adder_x 
		point.pose.position.y = msg.markers[0].pose.position.y + adder_y 
		point.header.stamp = rospy.Time.now()
		point.header.frame_id = "map"
		self._pub.publish(point)
		#rospy.loginfo(point)
		time.sleep(3)
		"""
	
	def run(self):
		"""Just wait for callbacks or interupt
		"""
		rospy.loginfo("Start converting")
		rospy.spin()

"""

if __name__ == "__main__":
	# Init ros, create and, run the node
	rospy.init_node("ht_marker2stamp")
	ht2st = Ht_marker2point()
	try:
		ht2st.run()
	except rospy.ROSInterruptException:
		pass 
		
"""

class Test(object):
	def __init__(self):
		human_velocity_list = []
		velocity = [(0.00123, 0.00234), (0.00321, 0.00432), (0.000511, 0.000622), (0.0001, 0.0002)]
		value = [[0, 0, 0] for i in range(3)]
		time = 0
		for i in range(20):
			t = np.random.normal(loc=0, scale=0.3, size=1)
			a = np.random.normal(loc = 0, scale=0.001, size=2)
			time += 13 + t[0]
			if i == 13:
				value.append([time, 12, 13])
				value[1] = [time, 5, 5]
				print("add + change")
				
			for j, v in enumerate(value):
				value[j][0] = time
				value[j][1] = velocity[j][0] * 13 + v[1] + a[0]
				value[j][2] = velocity[j][1]  * 13 + v[2] + a[1]
			if i >= 11:
				print(list(map((lambda x: list(map(lambda x: math.floor(x * 100) / 100, x))), value)))

			for j in range(len(value)):
				try:
					v = human_velocity_list[j].append_and_pop(value[j])
				except IndexError:
					print("Detection for new client")
					human_velocity_list.append(Human_velocity(value[j]))
					v = (0, 0)
				print("ID" + str(j) + ":" + str(list(map(lambda x: (math.floor(x*1000)/1000), v))))
				
			
		

		

	def _in_callback(self, msg:MarkerArray):
		n = len(msg.markers) // 2
		for i in range(n):
			x = msg.markers[i*2].pose.position.x
			y = msg.markers[i*2].pose.position.y
			msec = msg.markers[i*2].header####ToDo
			try:
				self.human_velocity_list[i].append_and_pop([msec, x, y])
			except IndexError:
				self.human_velocity_list.append(Human_velocity([msec, x, y]))

Test()