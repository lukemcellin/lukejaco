from __future__ import print_function
from openravepy import *
import numpy as np
from archierobot import ArchieRobot
#env = Environment() # create the environment



#env.Load('data/pr2test1.env.xml') # load a scene

from openrave_utils import *

xpos = [-0.4, -0.2, 0, 0.2, 0.4]
targets = np.array([0,0,0,0,0,0,0])

for index in range(0,1):
	
	xtarget = xpos[index]
	model_filename = 'jaco_dynamics'
	env, robot = initialize(model_filename)

	#env.SetViewer('qtcoin') # start the viewer

#robot = env.GetRobots()[0] # get the first robot
#robot = 

#robot_starting_dofs = np.array([-1, 2, 0, 2, 0, 4, 0, 1.11022302e-16,  -1.11022302e-16, 3.33066907e-16])


#gat = robot.GetActiveManipulator
	manip = robot.SetActiveManipulator(0) # set the manipulator to leftarm
	ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)

	if not ikmodel.load():
    		ikmodel.autogenerate()

	with env: # lock environment
	    Tgoal = np.array([[0.0, 0.0, 1.0, -0.4528/2],
  				       [-1.0, -1.0, 0.0, 0.0], #xtarget],
			               [1.0, 0.0, 1.0, 0.1], 
			               [0.0, 0.0,  0.0, 1.0]])
	    sol = manip.FindIKSolution(Tgoal, IkFilterOptions.CheckEnvCollisions) # get collision-free solution
	    with robot: # save robot state
	        robot.SetDOFValues(sol,manip.GetArmIndices()) # set the current solution
	        Tee = manip.GetEndEffectorTransform()
	        #env.UpdatePublishedBodies() # allow viewer to update new robot
        	#raw_input('press any key')
		#print(xtarget)
		#print(Tee)
		#print(sol, sep=',')
		targets = np.vstack((targets,sol))
print(targets, sep=",")
