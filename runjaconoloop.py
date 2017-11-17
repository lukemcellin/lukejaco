#! /usr/bin/env python
"""
This node demonstrates velocity-based PID control by moving the Jaco
so that it maintains a fixed distance to a target. 

Author: Andrea Bajcsy (abajcsy@eecs.berkeley.edu)
Based on: https://w3.cs.jmu.edu/spragunr/CS354_S15/labs/pid_lab/pid_lab.shtml
"""
import roslib; roslib.load_manifest('kinova_demo')

import rospy
import math
import pid
import tf
import sys, select, os
import thread
import argparse
import actionlib
import time
import trajopt_planner
import ros_utils
import openrave_utils

import kinova_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
from kinova_msgs.srv import *
from std_msgs.msg import Float32
from sympy import Point, Line

import numpy as np
from numpy import linalg
import matplotlib.pyplot as plt

from velsampling import *

prefix = 'j2s7s300_driver'

#last two are left and right pause trajs
buttonpos = np.array([[-3.09514739, 2.73559032, 0.0, -0.6919486,   0.0, 2.85564639, 1.51877286],
[-2.47415771e+00, 1.75537416e+00, 0.00000000e+00, -2.80533945e+00,  -7.10542736e-15, 1.72247170e+00, 2.23823127e+00],
[-2.89814609e+00, 2.17820321e+00, 0.00000000e+00, -1.81139492e+00,  -7.10542736e-15, 2.29358717e+00, 1.81424289e+00],
[2.85253513e+00, 2.17820321e+00, 0.00000000e+00, -1.81139492e+00,  -7.10542736e-15, 2.29358717e+00, 1.28173880e+00],
[2.43756859e+00, 1.75537416e+00, 0.00000000e+00, -2.80533945e+00,  -7.10542736e-15, 1.72247170e+00, 8.66772261e-01],
 [-2.48015012e+00, 2.43905671e+00, 0.00000000e+00, -1.11739605e+00, -7.10542736e-15, 2.72673255e+00, 2.23223886e+00],
[2.41355865e+00, 2.43905671e+00, 0.00000000e+00, -1.11739605e+00, -7.10542736e-15, 2.72673255e+00, 8.42762320e-01]])

buttonpos = np.mod(np.array(buttonpos),math.pi*2)

sequencematrix = np.genfromtxt('sequencematrix',delimiter = ",",dtype = int) # sequences for n trials
trialmatrix = np.genfromtxt('trialmatrix', delimiter = ",",dtype = int) #sequence number for each trial type... 1 = no delay low amp, 2 = no delay high amp, 3 = delay low amp, 4 = delay high amp

for trial in range(1,len(trialmatrix)): 
	
	print('Trial number:   ' ,trial)
	epsilon = 0.25 #make this larger in order to get rid of delay between taps
	MAX_CMD_TORQUE = 40.0

	seqtype = trialmatrix[trial,0] 
	trialtype = trialmatrix[trial,1]
	sequencelr =  sequencematrix[seqtype,:]
	sequencerl = fliplr(sequencematrix[seqtype,:])
	
	print ('Trial Type:  ', trialtype)
	#determine whether or not the robot should hesitate
	if trialtype > 2:
		fakepos = np.random.randint(5,7)
		fakepos = np.array([0,fakepos])
		sequencelr = np.concatenate((fakepos,sequencelr)) #adds a fake target position and then retracts back to start
		sequencerl = np.concatenate((fakepos,sequencerl))
	
	print('Start Trial')
	
	#add wait for enter press code
	#add start tone
	
	stroketime = np.empty([0,4])
	for seq in range(1,len(sequence)):
		if seq == 2: #robot will be at the start point
			choice = input('input choice' )
			if choice == 1:
				sequence = sequencelr 
			elif choice == 2:
				sequence = sequencerl
			
				
		class PIDVelJaco(object): 
			"""
			This class represents a node that moves the Jaco with PID control through ROS.
			The joint velocities are computed as:
		
				V = -K_p(e) - K_d(e_dot) - K_i*Integral(e)

			where:

				e = (target_joint configuration) - (current joint configuration)
				e_dot = derivative of error
				K_p = accounts for present values of position error
				K_i = accounts for past values of error, accumulates error over time
				K_d = accounts for possible future trends of error, based on current rate of change
	
			Subscribes to: 
				/j2s7s300_driver/out/joint_angles	- Jaco sensed joint angles
				/j2s7s300_driver/out/joint_torques	- Jaco sensed joint torques
	
			Publishes to:
				/j2s7s300_driver/in/joint_velocity	- Jaco commanded joint velocities 
			"""

			def __init__(self):
				"""
				Setup of the ROS node. Publishing computed torques happens at 100Hz.
				"""
				#time.sleep(1)

				startpos = sequence[seq-1]
				endpos = sequence[seq]		
			
				#use this to implement mid trajectory pauses			
				#pausepos = 0.3
				#pauseduration = 120
				# start admittance control mode
				self.start_admittance_mode()

				# ---- Trajectory Setup ---- #


				# determine whether or not to exaggerate amplitude... maybe i should write my own amplitude function
				if trialtype % 2 == 0:
				
					self.weights = -1 #highest amplitude
				else:
					self.weights = -1 #lowest amplitude

				# initialize start/goal based on task 
				start = buttonpos[startpos]
				goal = buttonpos[endpos]
				self.start = start
				self.goal = goal

				# create the trajopt planner and plan from start to goal
				self.planner = trajopt_planner.Planner()
				start_time = 0.0
				# total time for trajectory
				self.T = 2
				step_time = 0.1


				#stop first movement towards target early to create hesitation
				if startpos > 4 or endpos > 4:
					self.T = self.T/3 #shorten movement time accordingly

				# plan traj from start to goal 
				self.traj = self.planner.replan(self.start, self.goal, self.weights, start_time, self.T, step_time)

				#use below to manipulate velocity profile and pauses etc. 
	
				#if veltype == 1:
				#self.sampledtraj = velprofile(self.traj,0.5,8) 
				#elif veltype == 2:
				#	self.sampledtraj = velpause(self.traj,skew,kurt,pausepos,pauseduration)
				
				#self.sampledtraj = np.transpose(self.sampledtraj) #so array is in right order

				self.sampledtraj = self.traj

				# set the planner's trajectory (self.waypts) to be the sampledtraj
				self.planner.waypts = self.sampledtraj
				self.planner.num_waypts = len(self.sampledtraj)
			

				# plot the trajectory.... make sure viewers in openrave_utils are disabled so program can loop without crashing
				#openrave_utils.plotTraj(self.planner.env,self.planner.robot,self.planner.bodies,self.planner.waypts, size=6, color=[0, 0, 1])
	
				# save intermediate target position from degrees (default) to radians 
				self.target_pos = start.reshape((7,1))
				# save start configuration of arm
				self.start_pos = start.reshape((7,1))
				# save final goal configuration
				self.goal_pos = goal.reshape((7,1))

				# track if you have gotten to start/goal of path
				self.reached_start = False
				self.reached_goal = False

				# keeps running time since beginning of program execution
				self.process_start_T = time.time() 
				# keeps running time since beginning of path
				self.path_start_T = None 

				# ----- Controller Setup ----- #

				# stores maximum COMMANDED joint torques		
				self.max_cmd = MAX_CMD_TORQUE*np.eye(7)
				# stores current COMMANDED joint torques
				self.cmd = np.eye(7) 

				# P, I, D gains 
				p_gain = 100.0 #makes the robot move slower or faster
				i_gain = 0.0 
				d_gain = 20.0 #determines how faithful the robot is to the trajectory (I think)
				self.P = p_gain*np.eye(7)
				self.I = i_gain*np.eye(7)
				self.D = d_gain*np.eye(7)
				self.controller = pid.PID(self.P,self.I,self.D,0,0)

				# ---- ROS Setup ---- #

				rospy.init_node("runjaco")

				# create joint-velocity publisher
				self.vel_pub = rospy.Publisher(prefix + '/in/joint_velocity', kinova_msgs.msg.JointVelocity, queue_size=1)

				# create subscriber to joint_angles
				rospy.Subscriber(prefix + '/out/joint_angles', kinova_msgs.msg.JointAngles, self.joint_angles_callback, queue_size=1)

				# publish to ROS at 100hz
				r = rospy.Rate(100) 

				print "----------------------------------"
				print "Moving robot, press ENTER to quit:"
		
				while not rospy.is_shutdown():

					if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
						line = raw_input()
						break

					elif self.reached_goal:
						break
						stroketime[counter,0] = #whatever function returns time

					self.vel_pub.publish(ros_utils.cmd_to_JointVelocityMsg(self.cmd))
					r.sleep()
		
				print "----------------------------------"
	
				# end admittance control mode
				self.stop_admittance_mode()

			def start_admittance_mode(self):
				"""
				Switches Jaco to admittance-control mode using ROS services
				"""
				service_address = prefix+'/in/start_force_control'	
				rospy.wait_for_service(service_address)
				try:
					startForceControl = rospy.ServiceProxy(service_address, Start)
					startForceControl()           
				except rospy.ServiceException, e:
					print "Service call failed: %s"%e
					return None	

			def stop_admittance_mode(self):
				"""
				Switches Jaco to position-control mode using ROS services
				"""
				service_address = prefix+'/in/stop_force_control'	
				rospy.wait_for_service(service_address)
				try:
					stopForceControl = rospy.ServiceProxy(service_address, Stop)
					stopForceControl()           
				except rospy.ServiceException, e:
					print "Service call failed: %s"%e
					return None	

			def PID_control(self, pos):
				"""
				Return a control torque based on PID control
				"""
				error = -((self.target_pos - pos + math.pi)%(2*math.pi) - math.pi)
				return -self.controller.update_PID(error)

			def joint_angles_callback(self, msg):
				"""
				Reads the latest position of the robot and publishes an
				appropriate torque command to move the robot to the target
				"""
				# read the current joint angles from the robot
				curr_pos = np.array([msg.joint1,msg.joint2,msg.joint3,msg.joint4,msg.joint5,msg.joint6,msg.joint7]).reshape((7,1))

				# convert to radians
				curr_pos = curr_pos*(math.pi/180.0)	

				# update target position to move to depending on:
				# - if moving to START of desired trajectory or 
				# - if moving ALONG desired trajectory
				self.update_target_pos(curr_pos)

				# update cmd from PID based on current position
				self.cmd = self.PID_control(curr_pos)

				# check if each angular torque is within set limits
				for i in range(7):
					if self.cmd[i][i] > self.max_cmd[i][i]:
						self.cmd[i][i] = self.max_cmd[i][i]
					if self.cmd[i][i] < -self.max_cmd[i][i]:
						self.cmd[i][i] = -self.max_cmd[i][i]

			def update_target_pos(self, curr_pos):
				"""
				Takes the current position of the robot. Determines what the next
				target position to move to should be depending on:
				- if robot is moving to start of desired trajectory or 
				- if robot is moving along the desired trajectory 
				"""
				# check if the arm is at the start of the path to execute
				if not self.reached_start:

					dist_from_start = -((curr_pos - self.start_pos + math.pi)%(2*math.pi) - math.pi)			
					dist_from_start = np.fabs(dist_from_start)

					# check if every joint is close enough to start configuration
					close_to_start = [dist_from_start[i] < epsilon for i in range(7)]

					# if all joints are close enough, robot is at start
					is_at_start = all(close_to_start)

					if is_at_start:
						self.reached_start = True
						self.path_start_T = time.time()
					else:
						# if not at start of trajectory yet, set starting position 
						# of the trajectory as the current target position
						self.target_pos = self.start_pos
				else:
					t = time.time() - self.path_start_T

					# get next target position from position along trajectory
					self.target_pos = self.planner.interpolate(t)
					#print "target_pos: " + str(self.target_pos)

					# check if the arm reached the goal, and restart path
					if not self.reached_goal:
			
						dist_from_goal = -((curr_pos - self.goal_pos + math.pi)%(2*math.pi) - math.pi)			
						dist_from_goal = np.fabs(dist_from_goal)

						# check if every joint is close enough to goal configuration
						close_to_goal = [dist_from_goal[i] < epsilon for i in range(7)]
			
						# if all joints are close enough, robot is at goal
						is_at_goal = all(close_to_goal)
			
						if is_at_goal:
							self.reached_goal = True
					else:
						#print "REACHED GOAL! Holding position at goal."
						self.target_pos = self.goal_pos
						print(t)

		if __name__ == '__main__':
			PIDVelJaco()
	
