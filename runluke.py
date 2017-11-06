#! /usr/bin/env python
import math
import sys, select, os
import thread
import argparse
import actionlib
import time
import trajopt_planner

import numpy as np
from numpy import linalg
import matplotlib.pyplot as plt

import openrave_utils
from openrave_utils import *
from velsampling import *


#pick= [184.2, 151.6, 183.8, 101.8, 224.2, 216.9, 310.8]
#place = [210.8, 101.6, 192.0, 114.7, 222.2, 246.1, 322.0]




#reverse sign of 1 4,6 7 so animation isn't upside down

buttonpos = np.array([[3.08414056, 2.11856419, 0.0/math.pi*2,  -0.61303322,  0.0, -2.73159741, 1.51334423],
 [-2.53191311e+00, 1.74561523e+00, 0.00000000e+00/math.pi*2, -2.48885741e+00,  -7.10542736e-15, 2.04871267e+00, 2.18047587e+00],
 [-2.81503678e+00, 1.97147218e+00, 0.00000000e+00/math.pi*2, -1.90069778e+00,   -7.10542736e-15, 2.41101535e+00, 1.89735220e+00],
 [2.36385650e-02, -2.02706932e+00, 0.00000000e+00/math.pi*2, 1.73870882e+00,
    0.00000000e+00, -2.51740716e+00, -1.54715776e+00],
 [2.77060652e+00, 1.97147218e+00, 0.00000000e+00/math.pi*2, -1.90069778e+00,
   -7.10542736e-15, 2.41101535e+00, 1.19981020e+00],
 [2.49367962e+00, 1.74561523e+00, 0.00000000e+00/math.pi*2, -2.48885741e+00,
   -7.10542736e-15, 2.04871267e+00, 9.22883296e-01]])

buttonpos = np.mod(np.array(buttonpos),math.pi*2)


	#loop for whole sequence
seqtraj = np.empty((0,7))
sequence = [0,1,5,2,4,5,1]
for index in range(1,len(sequence)):
	sindex = sequence[index-1]
	start = buttonpos[sindex,:]
       	eindex = sequence[index]
	goal = buttonpos[eindex,:]

	if __name__ == '__main__':

		#input relevant stuff
		veltype = input('1 = no pause, 2 = pause')
		skew = input('0.5 = symmetry between acel & decel')
		kurt = input('1-20: sharpness of acel & decel')
		pausepos = input('from 0-1: when hand will pause')
		pausetime = input('amount of frames hand will pause')


	# create the trajectory optimization-based planner
		planner = trajopt_planner.Planner()

	# max table weight = 0, which means robot will prioritize staying close to the table
	# See what happens when you change it to 1!
		table_weight = 0
	
	# set the start time of the traj, final time and step time (in seconds)
		start_time = 0.0
		final_time = 15
		step_time = 0.5

		# plan a trajectory from start to goal, with the feature weight
		#goal = start
		#start = goal
		traj = planner.replan(start, goal, table_weight, start_time, final_time, step_time)		

		
		#take traj and transform it to desired velocity profile		
		if veltype == 1:
			sampledtraj = velprofile(traj,skew,kurt)
			sampledtraj = np.transpose(sampledtraj)

		elif veltype == 2:
			sampledtraj = velpause(traj,skew,kurt,pausepos,pausetime)
			sampledtraj = np.transpose(sampledtraj)


		seqtraj = np.append(seqtraj,sampledtraj,axis=0)

		# plot the trajectory using functions from openrave_utils
	plotTraj(planner.env, planner.robot, planner.bodies, sampledtraj, color=[1,0,0]) #, increment=5)
	

		#np.savetxt('jacotraj', np.array(traj), delimiter=",")


	# sleep the sim for 20 seconds so the program doens't shut down
	time.sleep(1)
	try:
    		input("Press enter to continue")
	except SyntaxError:
    		pass
