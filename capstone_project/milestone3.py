import numpy as np
import core as mr
import data as dt

def FeedbackControl(X,Xd,Xd_next,Kp,Ki,delta_t):
    X_err=mr.MatrixExp6(mr.MatrixLog6(np.dot(mr.TransInv(X),Xd)))
    Vd=mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(X),Xd))*(1/delta_t))
    Xerr_intg=X_err*delta_t+Xerr_intg
    V=np.dot(mr.Adjoint(mr.MatrixLog6(np.dot(mr.TransInv(Xd),Xd_next))),Vd)\
    +np.dot(Kp,X_err)+np.dot(Ki,Xerr_intg)
    
