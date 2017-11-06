import scipy
import numpy as np
import skfuzzy

## sampling waypoints based on a sigmoid function gives us a bell shaped velocity profile... two parameters allow us to control 'skewness'(symmetry) and 'kurtosis'(sharpness of acceleration and deceleration) of velocity profile
 
def velprofile(traj,skew,kurt):

	samples = len(traj)
	allframes = 15000 #time in ms (one frame per ms)

	#create linear function to transform	
	x = range(1,101,1); 
	x = np.array(x)
	x = np.true_divide(x,100)

	#transform linear function to sigmoidal function
	index = skfuzzy.membership.sigmf(x,skew,kurt) 
	index = index*allframes
	index = np.rint(index)

	#interpolate up to 1000Hz (iterate for each joint)
	interpmatrix = np.empty((0,allframes))
	for i in range(0,7):
		a = traj[:,i]
		b = np.linspace(0,1,len(traj))
		interptraj = np.interp(np.linspace(0,1,allframes),b,a)
		interpmatrix = np.append(interpmatrix,[interptraj],axis=0)
	interpmatrix = np.transpose(interpmatrix)

	#interpolate sampling array for desired amount of samples
	a = index
	b = np.linspace(0,1,len(index))
	sampling = np.interp(np.linspace(0,1,samples),b,a) #interp samples
	sampling = np.rint(sampling) # round for indexing
	sampling = np.int_(sampling)

	#sample from interpolated trajs using sampling distribution
	sampledtraj = interpmatrix[sampling-1,:]
	sampledtraj = np.transpose(sampledtraj)

	return(sampledtraj)



## double sigmoid sampling, allows us to control skewness and kurtosis of velocity profile before and after the pause.... also allows us to control when to pause and for how long....

def velpause(traj,skew,kurt,pausepos,pausetime):

	samples = len(traj)
	allframes = 15000 #time in ms (one frame per ms)

	#generate sigmoid function for before pause
	x1 = range(1,101,1)
	x1 = np.true_divide(x1,100)
	x1 = skfuzzy.membership.sigmf(x1,skew,kurt) 
	size1 = round(len(x1)*pausepos)
	a = x1
	b = np.linspace(0,1,len(x1))
	index1 = np.interp(np.linspace(0,1,size1),b,a)#change size based on pausepos
	index1 = index1*pausepos

	#generate sigmoid function for after pause
	x2 = range(1,101,1) 
	x2 = np.true_divide(x2,100)
	x2 = skfuzzy.membership.sigmf(x2,skew,kurt) 
	size2 = round(len(x2)*(1-pausepos))
	a = x2
	b = np.linspace(0,1,len(x2))
	index2 = np.interp(np.linspace(0,1,size2),b,a)#change size based on pausepos
	index2 = index2*(1-pausepos)
	index2 = index2 + index1[len(index1)-1]

	#add pause
	pause = np.empty(pausetime)
	pause.fill(index1[len(index1)-1])

	#concatenate pause with movements before and after pause
	index = np.concatenate((index1,pause,index2))
	index = index*allframes
	index = np.rint(index)

	#interpolate up to 1000Hz (iterate for each joint)
	interpmatrix = np.empty((0,allframes))
	for i in range(0,7):
		a = traj[:,i]
		b = np.linspace(0,1,len(traj))
		interptraj = np.interp(np.linspace(0,1,allframes),b,a)
		interpmatrix = np.append(interpmatrix,[interptraj],axis=0)
	interpmatrix = np.transpose(interpmatrix)

	#interpolate sampling array for desired amount of samples
	a = index
	b = np.linspace(0,1,len(index))
	sampling = np.interp(np.linspace(0,1,samples),b,a) #interp samples
	sampling = np.rint(sampling) # round for indexing
	sampling = np.int_(sampling)

	#sample from interpolated trajs using sampling distribution
	sampledtraj = interpmatrix[sampling-1,:]
	sampledtraj = np.transpose(sampledtraj)

	return(sampledtraj)

