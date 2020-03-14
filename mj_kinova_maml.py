#!/usr/bin/env python3

###############
#Author: Yi Herng Ong
#Purpose: import kinova jaco j2s7s300 into Mujoco environment
#
#("/home/graspinglab/NearContactStudy/MDP/jaco/jaco.xml")
#
###############


import gym
import numpy as np
from mujoco_py import MjViewer, load_model_from_path, MjSim
import math
import matplotlib.pyplot as plt
import time 
class Kinova_MJ(object):
	def __init__(self):
		# self._model = load_model_from_path("/home/graspinglab/NearContactStudy/MDP/kinova_description/j2s7s300.xml")
		self._model = load_model_from_path("/home/kartik/.mujoco/j2s7s300/j2s7s300_maml.xml")
		
		self._sim = MjSim(self._model)
		self._viewer = MjViewer(self._sim)
		self._timestep = 0.0001
		self._sim.model.opt.timestep = self._timestep

		self._torque = [0,0,0,0,0,0,0,0,0,0]
		self._velocity = [0,0,0,0,0,0,0,0,0,0]

		self._jointAngle = [0,0,0,0,0,0,0,0,0,0]
		self._positions = [] # ??
		self._numSteps = 0
		self._simulator = "Mujoco"
		self._experiment = "" # ??
		self._currentIteration = 0

		# define pid controllers for all joints
#		self.pid = [PID_(0.1, 0.040, 0), PID_(1.9,0.06,0), PID_(1.1,0.060,0.0),PID_(0.1,0.040,0.0), PID_(0.1,0.040,0.0), PID_(0.1,0.040,0.0),PID_(0.1,0.040,0.0), PID_(0.1,0.0,0.0), PID_(0.1,0.0,0.0), PID_(0.1,0.0,0.0)]


	def set_step(self, seconds):
		self._numSteps = seconds / self._timestep
		# print(self._numSteps)

	# might want to do this function on other file to provide command on ros moveit as well
	def set_target_thetas(self, thetas): 
		self.pid[0].set_target_jointAngle(thetas[0])
		self.pid[1].set_target_jointAngle(thetas[1])
		self.pid[2].set_target_jointAngle(thetas[2])
		self.pid[3].set_target_jointAngle(thetas[3])
		self.pid[4].set_target_jointAngle(thetas[4])
		self.pid[5].set_target_jointAngle(thetas[5])
		self.pid[6].set_target_jointAngle(thetas[6])
		self.pid[6].set_target_jointAngle(thetas[7])
		self.pid[6].set_target_jointAngle(thetas[8])

		# print("joint1",self.pid[1]._targetjA)


	def run_mujoco(self,thetas = [2, 1, 0.1, 0.75, 4.62, 4.48, 4.88, 0.0, 0.0, 0.0]):
#		thetas[0] = 3.14
#		thetas[1] = -1
#		thetas[2] = 1.5 
#		thetas[3] = 2.5
     
		self._sim.data.qpos[0:10] = thetas[:] 		#first 10 - first 7 are joint angles, next 3 are finger pose
		self._sim.forward()
#		img = self._sim.render(width=480,height=640,camera_name="camera")
		img = self._sim.render(width=224,height=224,camera_name="camera")
#		plt.imsave("img_lets_see_gt_v2.png",img)
		return img
#		for i in range(1000):
#			self._viewer.render()
			
		
#		plt.pause(0.1)
#		plt.show()

#		self.set_target_thetas(thetas_0)
#		# print("joint1",self.pid[1]._targetjA)
#		for step in range(int(self._numSteps)):
#
#			if step > 1000:
#				for i in range(9):
#					# thetas_0[0] = 0.3
#					if i > 5:
#						thetas_0[i] += 0.01
#					self.pid[i].set_target_jointAngle(thetas_0[i])
#					self._jointAngle[i] = self._sim.data.qpos[i]
#					self._torque[i] = self.pid[i].get_Torque(self._jointAngle[i])
#					self._sim.data.ctrl[i] = self._torque[i]
#
#			if step < 200:
#				for joint in range(9):
#					self._jointAngle[joint] = self._sim.data.qpos[joint]
#					# print(self._jointAngle[joint])
#					self._torque[joint] = self.pid[joint].get_Torque(self._jointAngle[joint])
#					self._sim.data.ctrl[joint] = self._torque[joint]
#
#			if step > 200 and step < 600:
#				for k in range(9):
#					# thetas_0[0] = 0.3
#					# thetas_0[1] = 0.5
#					thetas_0[2] = 0.1
#					thetas_0[4] = 0.3
#					thetas_0[5] = -0.7
#					self.set_target_thetas(thetas_0)
#					self._jointAngle[k] = self._sim.data.qpos[k]
#					# print(self._jointAngle[joint])
#					self._torque[k] = self.pid[k].get_Torque(self._jointAngle[k])
#					self._sim.data.ctrl[k] = self._torque[k]
#
#			if step > 600 and step < 700:
#				for k in range(9):
#					thetas_0[0] = -0.3
#					self.set_target_thetas(thetas_0)
#					self._jointAngle[k] = self._sim.data.qpos[k]
#					# print(self._jointAngle[joint])
#					self._torque[k] = self.pid[k].get_Torque(self._jointAngle[k])
#					self._sim.data.ctrl[k] = self._torque[k]
#
#			if step > 700 and step < 800:
#				for k in range(9):
#					# thetas_0[1] += 0.05
#					thetas_0[1] = 0.25
#					self.set_target_thetas(thetas_0)
#					self._jointAngle[k] = self._sim.data.qpos[k]
#					# print(self._jointAngle[joint])
#					self._torque[k] = self.pid[k].get_Torque(self._jointAngle[k])
#					self._sim.data.ctrl[k] = self._torque[k]	
#
#			if step > 800 and step < 900:
#				for k in range(9):
#					thetas_0[0] = 0.5
#					# thetas_0[1] = 0.25
#					self.set_target_thetas(thetas_0)
#					self._jointAngle[k] = self._sim.data.qpos[k]
#					# print(self._jointAngle[joint])
#					self._torque[k] = self.pid[k].get_Torque(self._jointAngle[k])
#					self._sim.data.ctrl[k] = self._torque[k]	
#
#			if step > 900 and step < 1000:
#				for k in range(9):
#					# thetas_0[0] = 0.5
#					thetas_0[1] = 0.5
#					self.set_target_thetas(thetas_0)
#					self._jointAngle[k] = self._sim.data.qpos[k]
#					# print(self._jointAngle[joint])
#					self._torque[k] = self.pid[k].get_Torque(self._jointAngle[k])
#					self._sim.data.ctrl[k] = self._torque[k]
#			# self._positions.append([self._sim.data.get_body_xpos('j2s7s300_link_7')[0], self._sim.data.get_body_xpos('j2s7s300_link_7')[1], self._sim.data.get_body_xpos('j2s7s300_link_7')[2], self._sim.data.get_body_xquat('j2s7s300_link_7')[0], self._sim.data.get_body_xquat('j2s7s300_link_7')[1], self._sim.data.get_body_xquat('j2s7s300_link_7')[2], self._sim.data.get_body_xquat('j2s7s300_link_7')[3]])
#			# self._sim.data.qpos[0:10] = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
#
#			self._sim.step()
#			self._viewer.render()
#			if step == 10:
#				img = self._sim.render(width=1000,height=1000,camera_name="camera")
#				plt.imshow(img)
#				plt.pause(100)
#                
#				print(img.shape)





	

if __name__ == '__main__':
	sim = Kinova_MJ()
	for i in range(1):
		sim.run_mujoco()
		time.sleep(0.5)
		print("Next")
