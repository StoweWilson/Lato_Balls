import math
import numpy as np

theata = 20
vi = 0
g = 9.8
deltaT= 0.01
t = 0.01
xi =0
yi = 0
l =0
counter = 0
while theata >= 0:
    ax = g*(math.sin(theata))
    vx = vi + (ax * deltaT)
    xf = xi -((math.cos(theata))* (vx * deltaT))
    yf = yi -((math.sin(theata))* (vx * deltaT))
    #new theata
    t = t + deltaT
    print(t)
    theata = np.arctan(xf/yf)
    xi = xf
    yi = yf

    
