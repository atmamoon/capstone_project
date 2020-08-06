import numpy as np

def NextState(current_config,controls,delta_t,speed_limits=[0,0]):
    next_config=np.zeros(12)
    for i in range(9):
        if controls[i]>speed_limits[1]:
            controls[i]=speed_limits[1]
        elif controls[i]<speed_limits[0]:
            controls[i]=speed_limits[0]
    print(controls)
    for i in range(3,12):
        next_config[i]=current_config[i]+controls[i-3]*delta_t
    l=0.47/2
    w=0.3/2
    r=0.0475
    F=np.array([[-1/(l+w),1/(l+w),1/(l+w),-1/(l+w)],[1,1,1,1],[-1,1,-1,1]])*r/4
    Vb=np.dot(F,controls[5:9]*delta_t)
    if Vb[0]!=0:
        delta_qb=np.array([Vb[0],(Vb[1]*np.sin(Vb[0])+Vb[2]*(np.cos(Vb[0])-1))/Vb[0],(Vb[2]*np.sin(Vb[0])+Vb[1]*(-np.cos(Vb[0])+1))/Vb[0]])
    else:
        delta_qb=Vb
    delta_q=np.dot(np.array([[1,0,0],[0,np.cos(next_config[0]),-np.sin(next_config[0])],[0,np.sin(next_config[0]),np.cos(next_config[0])]]),delta_qb)
    for i in range(0,3):
        next_config[i]=current_config[i]+delta_q[i]
    #print(next_config)
    #print(Vb)
    return next_config



speed=np.array([4.5,4.5,4.5,4.5,4.5,10,10,10,10])
limits=[-15,15]
initial_config=np.zeros(12)
configuration=[]
for i in range(100):
    if i<63:
        initial_config=NextState(initial_config,speed,0.01,limits)
        initial_config=np.append(initial_config,[1])
        configuration.append(initial_config)
    else:
        initial_config=NextState(initial_config,speed,0.01,limits)
        initial_config=np.append(initial_config,[0])
        configuration.append(initial_config)
configuration=np.array(configuration)
np.savetxt("milestone1.csv",configuration,delimiter=',')
