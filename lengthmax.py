import math
import matplotlib.pyplot as plt

i=1 #indexing variable
g=9.8 #gravity
theta=0.25 #initial angular position
thetadot=0 #initial angular velocity
l=0.1 #stringlength
dt=0.01 #time stamp
m=0.022 #mass
k=0.00035 #drag coeficcient
Max = 0
L = 0
ZR=0
Z=0

omega=2*math.sqrt(g/l)-2
max_omega=2*math.sqrt(g/l)-2 #angular frequency of the oscilator
A=0.08 #amplitude of the oscilator
C=0.88 #coeficient of restitution
m=0.022 #mass
t = [] 
T = []# Initialize t as a list
t(i)==0 # time
T(i)==theta #angle
j=1
domega=0.1
min_time=10

thetadotdot=(-m*g*math.sin(theta)-k*thetadot+m*A*l*omega**2*math.cos(omega*t(i))*math.sin(theta))/(m*l) #calculating initial angular acceleration
R=0

dl=0.02

while (l<0.5):

    while (omega<2*math.sqrt(g/l)+2):

        while (i<10000):
            if(theta<0.01):
                thetadot-=thetadot*C
                theta=0.01
                
            elif(theta>0.01):
                Z=0
                if (thetadot<0):
                    Z=1
                    if (R == 0):
                        break
                    
                if (math.cos(omega*t(i))>0):
                    R=1
                    
            if(ZR==R*Z and thetadot==thetadot+thetadotdot*dt and theta==theta+thetadot*dt and thetadotdot==(-m*g*math.sin(theta)-k*thetadot-ZR*m*A*l*omega^2*math.cos(omega*t(i))*math.sin(theta))/(m*l)
and T(i+1)==theta and t(i+1)==t(i)+dt):
                break
        i=i+1

    if (theta > 1):
        break
   



    if (t(i-1)<min_time):
        max_omega=omega
        min_time=t(i-1)
    if( omega==omega+domega and i==1 and t(i)==0 and theta==0.25 and thetadot==0 and thetadotdot==(-m*g*math.sin(theta)-ZR*k*thetadot-m*A*l*omega^2*cos(omega*t(i))*sin(theta))/(m*l)):
        break



i=1
t(i)==0
theta=0.25
thetadot=0
thetadotdot=(-m*g*math.sin(theta)-k*thetadot-ZR*m*A*l*omega^2*math.cos(omega*t(i))*math.sin(theta))/(m*l)
l=l+dl

L(j)==l
Max(j)==max_omega
omega==2*math.sqrt(g/l)-2
max_omega==2*math.sqrt(g/l)-2
t(i)==0 # time
T(i)==theta #angle
j=j+1


plt.plot(L,Max)
plt.xlabel('Length')
plt.ylabel('Max Omega')

