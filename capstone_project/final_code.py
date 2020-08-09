import numpy as np
import core as mr
import data as dt

import milestone1 as m1
import milestone2 as m2
import milestone3 as m3

def rearrange_to_se3(T):
    mat=[[T[i],T[i+1],T[i+2]] for i in range(0,9,3)]
    mat=np.array(mat)
    mat=np.append(mat,[[T[9]],[T[10]],[T[11]]],axis=1)
    mat=np.append(mat,[[0,0,0,1]],axis=0)
    return mat

def controls_rearrange(u):
    controls=np.zeros(9)
    for i in range(0,9):
        if i<5:
            controls[i]=u[i+4]
        else:
            controls[i]=u[i-5]
    return controls

T0e=mr.FKinBody(dt.M0e,dt.B_list,dt.Theta_list)
Tse_current=np.dot(dt.Tsb,np.dot(dt.Tb0,T0e))
RT=TrajectoryGenerator(Tse_current,dt.Tsc_initial,dt.Tsc_final,dt.Tce_grasp,dt.Tce_standoff)
N,m=np.shape(RT)
Kp=0
Ki=0
delta_t=0
speed_limit=[-15,15]
final_configuration=[]

def request_theta()
for i in range(N-1):
    Xd=rearrange_to_se3(RT[i])
    Xd_next=rearrange_to_se3(RT[i+1])
    U=m3.FeedbackControl(Tse_current,Xd,Xd_next,Kp,Ki,delta_t)
    new_configuration=m1.NextState(current_config,controls_rearrange(U),delta_t,speed_limit)
    new_configuration=np.append(new_configuration,[RT[12]])
    final_configuration.append(new_configuration)
