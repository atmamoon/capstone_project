import numpy as np
import core as mr
import data as dt

import milestone1 as m1
import milestone2 as m2
import milestone3 as m3


Tse=mr.FKinBody(dt.M0e,dt.B_list,dt.Theta_list)
RT=TrajectoryGenerator(Tse_initial,Tsc_initial,Tsc_final,Tce_grasp,Tce_standoff)
