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
x0 = 10                             # desired position in m
kp = 1                             # proportional gain
## ki = 1                           # integral gain
kd = 1                            # derivative gain

t0=0                              #initial time
tf=1                              #final time
h=0.0001                          # time step
x = 1                             # initialize position
#r = x0                            # reference signal
r = np.sin(x)                    # reference signal
e = x-x0                          # define error
e_d = 0                           # initialize error velocity
e_dd = 0                          # initialize error acceleration

def f(t,y):                       #function representing the state space error dynamics
    A = np.array([[0,1],[(-kp-k)/m,(-c-kd)/m]])
    f = np.dot(A,y) 
    return f

def solver(t,y,h):                #RK4 solver
    k1 = f(t,y)
    k2 = f(t+h/2,y+h*k1/2)
    k3 = f(t+h/2,y+h*k2/2)
    k4 = f(t+h,y+h*k3)
    
    y = y + h*(k1+2*k2+2*k3+k4)
    return y

#----------main program----------#
t=t0
time = [t]                        #list to store the time values
y = np.array([[e],[e_d]])         #initialize error state
pos_error = [y[0]]                #initialize array to store position error values 
position = [x]                    #initialize array to store position values
#count1 = 0
#count2 = 0
#rise_t = 0
#settling_t = 0
while t<tf:                       
    y = solver(t,y,h)             #solver to give the next state 
    t = t+h                       #update time
    x = y[0]+r                    #update position
    #print(x)
    u = -kp*y[0]-kd*y[1]          #control input
    F = k*x0 + u                  #force
    #store time, position and position_error in an array
    time.append(t)                
    pos_error.append(y[0])        
    position.append(x)     
    #print(F)
    print(y[0])
    #print(np.sin(x))
    
plt.plot(time, pos_error)
#plt.plot(time,position) 
plt.xlabel('time (in sec), h=0.0001 sec') 
plt.ylabel('position error (in m)') 
plt.title('PD controller with kp=1 and kd=1') 
plt.show() 

'''
max_error = 0

for i in range(int(tf/h)):
    if error[i]==0:
        count1 = count1 + 1
        if count1==1:
            rise_t=time[i]
    
    temp = np.array(error)
    temp = temp[i:int(tf/h)]
    
    if abs(max(temp)/x0)<0.05:
        settling_t = time[i]
        break

print("rise time is very small, nearly equal to ",rise_t)
print("settling time is ",settling_t," sec")
'''




