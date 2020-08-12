import matplotlib.pyplot as plt
import numpy as np
'''
class error:
    total_time=0
    def __init__(self,e):
        self.error_value=e
        total_time+=0.01
        self.time=error.total_time'''

x=[]
plot=np.genfromtxt("error.csv",delimiter=',')
N,m=np.shape(plot)
t=np.linspace(0,N*0.01,N)
for j in range(m):
    pointers=[]
    for i in range(N):
        pointers.append(plot[i,j])
    #error.total_time=0
    x.append(pointers)


for k in x:
    plt.plot(t,k,'r')
plt.title('new')
plt.show()


plt.plot(x,y, label='Loaded from file!')
plt.xlabel('x')
plt.ylabel('y')
plt.title('Interesting Graph\nCheck it out')
plt.legend()
plt.show()
