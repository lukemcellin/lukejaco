def velprofile(jacotraj):

	import scipy
	import numpy as np
	import skfuzzy



	samples = 1500;
	allframes = 15000; #time in ms (one frame per ms)

	# create desired function for velocity profile

	x = range(1,101,1); #create linear function to transform
	x = np.array(x)
	x = np.true_divide(x,100)

	index = skfuzzy.membership.sigmf(x,0.5,10); #sigmoid for bell shaped velocity
	index = index*allframes
	index = np.rint(index);

	#index = log; #logit for fast movement with slowing in the middle

	###for double sigmoid for stop and start

	#pausepos = 0.5; #determine at what stage of the action robot will pause

	#before pause
	#x1 = frange(0.1,10,0.1); 
	#x1 = skfuzzy.membership.sigmf(x1,2,5); 
	#size1 = round(len(x1)*pausepos);
	#index1 = interp1(linspace(0,1,length(x1)),x1(1:length(x1),:),linspace	(0,1,size1)); CHANGE TO PYTHON SYNTAX
	#index1 = index1*pausepos;

	#after pause
	#x2 = frange(0.1,10,0.1)
	#x2 = skfuzzy.membership.sigmf(x2,2,5); 
	#size2 = round(len(x2)*(1-pausepos));
	#index2 = interp1(linspace(0,1,length(x2)),x1(1:length(x2),:),linspace	(0,1,size2)); CHANGE TO PYTHON SYNTAX
	#index2 = index2*(1-pausepos); index2 = index2 + index1(length	(index1));

	#add pause
	#pausesize = 2;
	#pause(1:pausesize) = index1(len(index1));

	#index = numpy.concatenate((index1,pause,index2));
	#index = index*allframes; 
	#index = int(round(index));

	interpmatrix = []
	#interpolate for one sample every 1ms
	#iterate for each joint
	for i in range(0,7):
		a = jacotraj[:,i]
		b = np.linspace(0,1,len(jacotraj))
		interptraj = np.interp(np.linspace(0,1,allframes),b,a)
		interpmatrix = np.append(interpmatrix,interptraj,axis=0)

	# interpolate desired amount of samples

	a = index
	b = np.linspace(0,1,len(index))
	sampling = np.interp(np.linspace(0,1,samples),b,a) #interp samples
	sampling = np.rint(sampling) # round for indexing

	sampledtraj = interptraj[sampling,:]
	return(sampledtraj)

	#sampling = scipy.interpolate.interp1d(index(:,1:len	(index)),numpy.linspace(0,1,samples));
