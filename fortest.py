import numpy as np
sequence = [0,3,1,4,5]
pos = np.array([[1,2,3],[4,5,6],[7,8,9],[10,11,12],[13,14,15],[16,17,18]])
for index in range(1,len(sequence)):
	sindex = sequence[index-1]
	startposition = pos[sindex,:]
        eindex = sequence[index]
	endposition = pos[eindex,:]
	print(index, startposition, endposition)
