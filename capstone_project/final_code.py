import numpy as np
import core as mr
import data as dt

import milestone1 as m1
import milestone2 as m2
import milestone3 as m3


Kp=2
Ki=1
delta_t=0.01
speed_limit=[-30,30]
Xerr_integral=[0,0,0,0,0,0]
U=[0,0,0,0,0,0,0,0,0]

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


def q_list(L):
    q_chassis=np.zeros(3)
    for i in range(3):
        q_chassis[i]=L[i]
    return q_chassis


def Theta_list(L,u=U):
    #joint_limits=[[-2.932,2.932],[-9999,9999],[-2.620,2.179],[-0.121,1.780],[-9999,9999]]
    #joint_limits=[[-9999,9999],[-2.5,2.67],[-2.163,2.179],[-1,1.780],[-9999,9999]]
    #joint_limits=[[-9999,9999],[-9999,9999],[-9999,9999],[-9999,9999],[-9999,9999]]
    #joint_limits=[[-2.932,2.932],[-1.117,1.553],[-2.620,2.530],[-1.780,1.780],[-9999,9999]]

    theta=np.zeros(5)
    for i in range(3,8):
        '''if L[i]>joint_limits[i-3][0] and L[i]<joint_limits[i-3][1]:# and (L[i]+u[i+1]*delta_t)>joint_limits[i-3][0]\
        #and (L[i]+u[i+1]*delta_t)<joint_limits[i-3][1]:
            theta[i-3]=L[i]
        else:
            theta[i-3]=0'''
        theta[i-3]=L[i]
    return theta



final_configuration=[]
#initial_configuration=[0,-0.210,0,0,0.714,-2.054,1.426,0,0,0,0,0,0]
initial_configuration=[0.6,-0.210,0,0,0,-0,0,0,0,0,0,0,0]
final_configuration.append(initial_configuration)

q=q_list(final_configuration[-1])
Tsb=np.array([[np.cos(q[0]),-np.sin(q[0]),0,q[1]],[np.sin(q[0]),np.cos(q[0]),0,q[2]],[0,0,1,0.0963],[0,0,0,1]])
T0e=mr.FKinBody(dt.M0e,dt.B_list,Theta_list(final_configuration[-1]))
#Tse_current=np.dot(Tsb,np.dot(dt.Tb0,T0e))
Tse_current=np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0.5],[0,0,0,1]])
RT=m2.TrajectoryGenerator(Tse_current,dt.Tsci,dt.Tsc_goal,dt.Tcef,dt.Tce_stand)
N,m=np.shape(RT)


for i in range(N-1):

    q=q_list(final_configuration[-1])
    Tsb=np.array([[np.cos(q[0]),-np.sin(q[0]),0,q[1]],[np.sin(q[0]),np.cos(q[0]),0,q[2]],[0,0,1,0.0963],[0,0,0,1]])
    T0e=mr.FKinBody(dt.M0e,dt.B_list,Theta_list(final_configuration[-1]))
    Tse_current=np.dot(Tsb,np.dot(dt.Tb0,T0e))
    Xd=rearrange_to_se3(RT[i])
    Xd_next=rearrange_to_se3(RT[i+1])
    U,Xerr_integral=m3.FeedbackControl(Tse_current,Xd,Xd_next,Kp,Ki,delta_t,Theta_list(final_configuration[-1],controls_rearrange(U)),Xerr_integral)
    new_configuration=m1.NextState(final_configuration[-1],controls_rearrange(U),delta_t,speed_limit)
    if i==N-2:
        print(Xd_next)
        #print(new_configuration)
    new_configuration=np.append(new_configuration,[RT[i][12]])
    final_configuration.append(new_configuration)

print(Xerr_integral)
print(Tse_current)
#print(final_configuration[-1])
np.savetxt("final.csv",final_configuration,delimiter=',')
