import numpy as np
import data as dt
import core as mr
T=np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0.65],[0,0,0,1]])
theta=mr.IKinBody(dt.B_list,dt.M0e, T,[0,0,0.3,0.5,0.1],0.01,0.001)
print(theta)
