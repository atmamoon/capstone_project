'''
MOBILE MANIPULATION CAPSTONE

SK MOHAMMED MAMOON MONDAL

PLOTS THE ERROR CURVE WITH RESPECT TO TIME WITH MATPLOTLIB PYPLOT
'''
import matplotlib.pyplot as plt
import numpy as np


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
