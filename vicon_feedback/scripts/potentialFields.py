#!/usr/bin/env python3

import numpy as np 
import matplotlib.pyplot as plt

x = np.linspace(0,20,100)
print(np.shape(x))
id = np.arange(0,1000)

y = np.zeros(0)

thresh = 1.0
for i in range(len(x)):
	val = x[i]%10
	if (np.abs(val) < thresh):
		y = np.append(y,i)
		plt.scatter(x[i],val,color='r')
	elif np.abs(10-val) < thresh:
		plt.scatter(x[i],val,color='b')


#plt.scatter(y.transpose())
plt.show()