import numpy as np
import core as mr

def rearrange(mat,gripper=0):
    '''
    ordered_mat=[]
    for i in mat:
        ordered_mat.append([i[k,j] for k in range(3) for j in range(4)])
    print(ordered_mat)
    return ordered_mat'''
    ret=[mat[k,j] for k in range(3) for j in range(3)]
    ret.append(mat[0,3])
    ret.append(mat[1,3])
    ret.append(mat[2,3])
    ret.append(gripper)
    return ret


def TrajectoryGenerator(Tse_initial,Tsc_initial,Tsc_final,Tce_grasp,Tce_standoff):
    Xse=[]
    d=10
    N=400
    Tf=4
    method=3
    Tse_final=Tsc_initial
    #Tse_final[2,3]=Tsc_initial[2,3]-d
    Tse_final=np.dot(Tsc_initial,Tce_standoff)
    X=mr.ScrewTrajectory(Tse_initial,Tse_final,Tf,N,method)
    for i in X:
        Xse.append(rearrange(i))


    Tse_initial=X[-1]
    Tse_final=np.dot(Tsc_initial,Tce_grasp)
    X=mr.ScrewTrajectory(Tse_initial,Tse_final,Tf,N,method)
    for i in X:
        Xse.append(rearrange(i))


    Tse_initial=X[-1]
    for i in range(63):
        Xse.append(rearrange(Tse_initial,1))


    Tse_initial=X[-1]
    Tse_final=np.dot(Tsc_initial,Tce_standoff)
    X=mr.ScrewTrajectory(Tse_initial,Tse_final,Tf,N,method)
    for i in X:
        Xse.append(rearrange(i,1))


    Tse_initial=X[-1]
    Tse_final=np.dot(Tsc_final,Tce_standoff)
    X=mr.ScrewTrajectory(Tse_initial,Tse_final,Tf,N,method)
    for i in X:
        Xse.append(rearrange(i,1))


    Tse_initial=X[-1]
    Tse_final=np.dot(Tsc_final,Tce_grasp)
    X=mr.ScrewTrajectory(Tse_initial,Tse_final,Tf,N,method)
    for i in X:
        Xse.append(rearrange(i,1))


    Tse_initial=X[-1]
    for i in range(63):
        Xse.append(rearrange(Tse_initial))


    Tse_initial=X[-1]
    Tse_final=np.dot(Tsc_final,Tce_standoff)
    X=mr.ScrewTrajectory(Tse_initial,Tse_final,Tf,N,method)
    for i in X:
        Xse.append(rearrange(i))

    print(Xse)
    np.savetxt("milestone2.csv",Xse,delimiter=',')

q=[0,0,0]
d3=0.043
Tsb=np.array([[np.cos(q[0]),-np.sin(q[0]),0,q[1]],[np.sin(q[0]),np.cos(q[0]),0,q[2]],[0,0,1,0.0963],[0,0,0,1]])
Tb0=np.array([[1,0,0,0.1662],[0,1,0,0],[0,0,1,0.0026],[0,0,0,1]])
M0e=np.array([[1,0,0,0.033],[0,1,0,0],[0,0,1,0.6546],[0,0,0,1]])
#Tse=np.dot(Tsb,np.dot(Tb0,M0e))

Tse=np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0.65],[0,0,0,1]])
Tsci=np.array([[1,0,0,1],[0,1,0,0],[0,0,1,0.025],[0,0,0,1]])
Tsc_goal=np.array([[0,1,0,0],[-1,0,0,-1],[0,0,1,0.025],[0,0,0,1]])
Tcef=np.array([[0,0,1,d3/2],[0,1,0,0],[-1,0,0,0],[0,0,0,1]])
Tce_stand=np.array([[0,0,1,d3/2],[0,1,0,0],[-1,0,0,0.10],[0,0,0,1]])
TrajectoryGenerator(Tse,Tsci,Tsc_goal,Tcef,Tce_stand)
