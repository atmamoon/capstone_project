'''
MOBILE MANIPULATION CAPSTONE

SK MOHAMMED MAMOON MONDAL

lines starting with hash are comments

Only this file needs to be run to generate the required output
'''

#importing modules
import numpy as np
import core as mr
import data as dt

import milestone1 as m1
import milestone2 as m2
import milestone3 as m3


#adding Kp and Ki gains
Kp=2
Ki=1
#setting delta_t to 10ms
delta_t=0.01
speed_limit=[-30,30]

Xerr_integral=[0,0,0,0,0,0]
U=[0,0,0,0,0,0,0,0,0]


#function to rearrange_to_se3 from the 13 element vector configuration
def rearrange_to_se3(T):
    mat=[[T[i],T[i+1],T[i+2]] for i in range(0,9,3)]
    mat=np.array(mat)
    mat=np.append(mat,[[T[9]],[T[10]],[T[11]]],axis=1)
    mat=np.append(mat,[[0,0,0,1]],axis=0)
    return mat

#reaaranging controls because the order of wheel and arm velocities are different in milestone1 \
#   from the rest of the code
def controls_rearrange(u):
    controls=np.zeros(9)
    for i in range(0,9):
        if i<5:
            controls[i]=u[i+4]
        else:
            controls[i]=u[i-5]
    return controls

#returns the chassis configuration as 3 element vector
def q_list(L):
    q_chassis=np.zeros(3)
    for i in range(3):
        q_chassis[i]=L[i]
    return q_chassis

#returns a list of joint angles of the arm from the 13element configuration vector
def Theta_list(L):
    theta=np.zeros(5)
    for i in range(3,8):
        theta[i-3]=L[i]
    return theta


#initializing required lists
final_configuration=[]
#initial_configuration=[0,-0.210,0,0,0.714,-2.054,1.426,0,0,0,0,0,0]
initial_configuration=[0.6,-0.210,0,0,0,-0,0,0,0,0,0,0,0]
final_configuration.append(initial_configuration)

q=q_list(final_configuration[-1])
Tsb=np.array([[np.cos(q[0]),-np.sin(q[0]),0,q[1]],[np.sin(q[0]),np.cos(q[0]),0,q[2]],[0,0,1,0.0963],[0,0,0,1]])
T0e=mr.FKinBody(dt.M0e,dt.B_list,Theta_list(final_configuration[-1]))
#Tse_current=np.dot(Tsb,np.dot(dt.Tb0,T0e))
Tse_current=np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0.5],[0,0,0,1]])

#calling milestone2 to get the desired trajectory
RT=m2.TrajectoryGenerator(Tse_current,dt.Tsci,dt.Tsc_goal,dt.Tcef,dt.Tce_stand)
N,m=np.shape(RT)
Xerr=[]


#loops of the output of milestone2 and captures the actual new configuration for the youbot
for i in range(N-1):

    q=q_list(final_configuration[-1])
    Tsb=np.array([[np.cos(q[0]),-np.sin(q[0]),0,q[1]],[np.sin(q[0]),np.cos(q[0]),0,q[2]],[0,0,1,0.0963],[0,0,0,1]])
    T0e=mr.FKinBody(dt.M0e,dt.B_list,Theta_list(final_configuration[-1]))
    Tse_current=np.dot(Tsb,np.dot(dt.Tb0,T0e))
    Xd=rearrange_to_se3(RT[i])
    Xd_next=rearrange_to_se3(RT[i+1])
    U,Xerr_integral,err=m3.FeedbackControl(Tse_current,Xd,Xd_next,Kp,Ki,delta_t,Theta_list(final_configuration[-1]),Xerr_integral)
    new_configuration=m1.NextState(final_configuration[-1],controls_rearrange(U),delta_t,speed_limit)
    if i==N-2:
        print("desired configuration",Xd_next)
    new_configuration=np.append(new_configuration,[RT[i][12]])
    final_configuration.append(new_configuration)
    Xerr.append(err)

print("integral of the error",Xerr_integral)
print("current configuration",Tse_current)

#saves the final_configuration and error to csv files
np.savetxt("final.csv",final_configuration,delimiter=',')
np.savetxt("error.csv",Xerr,delimiter=',')
