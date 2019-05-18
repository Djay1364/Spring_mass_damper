# -*- coding: utf-8 -*-
"""
Created on Thu May 16 11:12:54 2019

@author: tiwar
"""
import numpy as np
import matplotlib.pyplot as plt

## constants                                    
m = 1.325                           # mass in Kg
k = 930                             # spring constant in N/m
c = 2.72                            # damping constant in N.s/m         
#x0 = 10                             # desired position in m
kp = 1                             # proportional gain
ki = 1000                             # integral gain
kd = 1                            # derivative gain

t0=0                                #initial time
tf=10                               #final time
h=0.0001                               # time step
x = 0                               # initialize position
x0 = np.cos(x)                             
e = x-x0                           # define error
e_d = 0                             # initialize error velocity
e_dd = 0                            # initialize error acceleration

def f(t,y):                         #function representing the state space error dynamics
    A = np.array([[0,1,0],[0,0,1],[-ki/m,(-kp-k)/m,(-c-kd)/m]])
    #B = np.array([[0],[1]])
    #u = np.array([-k*x0/m])
    f = np.dot(A,y) 
    return f

def solver(t,y,h):                  #RK4 solver
    k1 = f(t,y)
    k2 = f(t+h/2,y+h*k1/2)
    k3 = f(t+h/2,y+h*k2/2)
    k4 = f(t+h,y+h*k3)
    
    y = y + h*(k1+2*k2+2*k3+k4)
    return y

#----------main loop----------#
t=t0
time = [t]                         #list to store the time values
y = np.array([[e],[e_d],[e_dd]])   #initialize error state
error = [y[0]]                     #initialize array to store position error values
#count1 = 0
#count2 = 0
#rise_t = 1
#settling_t = 0
while t<tf:
    y = solver(t,y,h)              #solver to give the next state 
    t = t+h                        #update time
    #store time, position_error in an array
    time.append(t)                 
    error.append(y[0])     
    print(y[0])   
    
plt.plot(time, error) 
plt.xlabel('time (in sec), h=0.0001 sec') 
plt.ylabel('position error (in m)') 
plt.title('PID controller with kp=1, ki=1000, kd=1') 
plt.show() 

'''
for i in range(int(tf/h)):
    if abs(error[i]/x0)==0.05:
        count1 = count1 + 1
        if count1==1:
            rise_t=time[i]
    
    temp = np.array(error)
    temp = temp[i:int(tf/h)]
    
    if abs(max(temp)/x0)<0.0001:
        settling_t = time[i]
        break'''
'''
print("rise time is very small, nearly equal to ",rise_t)
print("settling time is ",settling_t," sec")
'''




