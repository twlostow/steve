#!/usr/bin/python

import sys
import time
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import griddata

lines=open(sys.argv[1],"rb").readlines()
n=0


points = []
values = []

while n < len(lines):
    tok = lines[n].split()
    n+=1
    if(tok[0] == 'r'):
	samples = int(tok[5])
	r_idx = int(tok[1])
	r_step = int(tok[3])
	r = r_idx * r_step
#	print(samples)
	for i in range(0,samples):
	    tok = lines[n].split()
	    rho = float(tok[0]) / 100
	    h = float(tok[1])
	    points.append((r, rho))
	    values.append(h)    
	    points.append((r, rho + 360))
	    values.append(h)    
	    points.append((r, rho - 360))
	    values.append(h)    
#	    print(rho, h)
	    n+=1
	

#print(points)
#print(values)
grid_r, grid_rho = np.mgrid[0:1280:100j,0:360:100j]
#print(grid_r)
#print(grid_rho)
#grid_rho, grid_r = np.mgrid[0:360:100j, 0:1:200j]
grid_z = griddata(points, values, (grid_r, grid_rho), method='linear')
#print(grid_z)

plt.imshow(grid_z.T, extent=(0,1280,0,360), origin='lower')
plt.colorbar()
#plt.gcf().set_size_inches(6, 6)
plt.show()