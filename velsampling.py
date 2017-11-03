#def velprofile(traj,samplingtype,skew,kurt):
def velprofile(traj):

	import scipy
	import numpy as np
	import skfuzzy


	#jacotraj = np.genfromtxt('jacotraj',delimiter =",")	
	jacotraj = traj
	#samples = len(jacotraj)
	samples = 31
	allframes = 15000 #time in ms (one frame per ms)

# create desired function for velocity profile

#sigmoidal function sampling
	
	#if samplingtype == 1
	#x = range(1,101,1); #create linear function to transform
	#x = np.array(x)
	#x = np.true_divide(x,100)

	#index = skfuzzy.membership.sigmf(x,skew,kurt)
	#index = skfuzzy.membership.sigmf(x,0.5,20) #sigmoid for bell shaped velocity
	#index = index*allframes
	#index = np.rint(index)

## logit function sampling

	#index = log; #logit for fast movement with slowing in the middle.... maybe invert the sigmoid somehow


## double sigmoid sampling for stop and start.... maybe find a smarter way to generate polynomial functions 

	#elif samplingtype == 2

	pausepos = 0.5 #determine at what stage of the action robot will pause

	#before pause
	x1 = range(1,101,1)
	x1 = np.true_divide(x1,100)
	x1 = skfuzzy.membership.sigmf(x1,0.5,15) 
	size1 = round(len(x1)*pausepos)
	a = x1
	b = np.linspace(0,1,len(x1))
	index1 = np.interp(np.linspace(0,1,size1),b,a)
	index1 = index1*pausepos

	#after pause
	x2 = range(1,101,1) 
	x2 = np.true_divide(x2,100)
	x2 = skfuzzy.membership.sigmf(x2,0.5,15) 
	size2 = round(len(x2)*(1-pausepos))
	a = x2
	b = np.linspace(0,1,len(x2))
	index2 = np.interp(np.linspace(0,1,size2),b,a)
	index2 = index2*(1-pausepos)
	index2 = index2 + index1[len(index1)-1]

	#add pause
	pausesize = 2
	pause = np.empty(pausesize)
	pause.fill(index1[len(index1)-1])


	index = np.concatenate((index1,pause,index2))
	index = index*allframes
	index = np.rint(index)

### interpolation and sampling

	interpmatrix = np.empty((0,allframes))
	#interpolate for one sample every 1ms
	#iterate for each joint
	for i in range(0,7):
		a = jacotraj[:,i]
		b = np.linspace(0,1,len(jacotraj))
		interptraj = np.interp(np.linspace(0,1,allframes),b,a)
		interpmatrix = np.append(interpmatrix,[interptraj],axis=0)

	interpmatrix = np.transpose(interpmatrix)

	# interpolate desired amount of samples

	a = index
	b = np.linspace(0,1,len(index))
	sampling = np.interp(np.linspace(0,1,samples),b,a) #interp samples
	sampling = np.rint(sampling) # round for indexing
	sampling = np.int_(sampling)
	sampledtraj = interpmatrix[sampling-1,:]
	sampledtraj = np.transpose(sampledtraj)

	return(sampledtraj)

