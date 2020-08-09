import numpy as np
import core as mr
import data as dt

def FeedbackControl(X,Xd,Xd_next,Kp,Ki,delta_t):
    tolerance=1e-10
    #Xerr_intg=np.zeros((4,4))
    Xerr_intg=np.zeros((1,6))
    X_err=mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(X),Xd)))
    Vd=mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(Xd),Xd_next))*(1/delta_t))
    Xerr_intg=X_err*delta_t+Xerr_intg
    #V=np.dot(mr.Adjoint(mr.MatrixLog6(np.dot(mr.TransInv(Xd),Xd_next))),Vd)\
    #+mr.se3ToVec(np.dot(Kp,X_err))+mr.se3ToVec(np.dot(Ki,Xerr_intg))
    V=np.dot(mr.Adjoint(np.dot(mr.TransInv(X),Xd)),Vd)\
    +Kp*X_err+Ki*Xerr_intg
    Jarm=mr.JacobianBody(dt.B_list,dt.Theta_list)
    Jbase=np.dot(mr.Adjoint(np.dot(mr.TransInv(mr.FKinSpace(dt.M0e,dt.B_list,dt.Theta_list))\
    ,mr.TransInv(dt.Tb0))),dt.F)
    J=np.append(Jbase,Jarm,axis=1)
    V=np.transpose(V)
    V[abs(V)<tolerance]=0.0
    u=np.dot(np.linalg.pinv(J),V)
    u[abs(u)<tolerance]=0
    print("error",X_err)
    print("Vd",Vd)
    print("V",V)
    print("adj V",mr.Adjoint(np.dot(mr.TransInv(X),Xd)))
    print("J",J)

    return u

X_d=np.array([[0,0,1,0.5],[0,1,0,0],[-1,0,0,0.5],[0,0,0,1]])
X_d_nxt=np.array([[0,0,1,0.6],[0,1,0,0],[-1,0,0,0.3],[0,0,0,1]])
X_=np.array([[0.170,0,0.985,0.387],[0,1,0,0],[-0.985,0,0.170,0.570],[0,0,0,1]])
'''Kp=np.identity(4)
Kp=Kp*0
Ki=np.identity(4)
Ki=Ki*0'''
Ki=0
Kp=1
deltat=0.01
print(FeedbackControl(X_,X_d,X_d_nxt,Kp,Ki,deltat))
