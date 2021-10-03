import numpy as np
import math as m

val = m.pi/180
cons = 180/m.pi

th = [10*val, 15*val, 0*val, -100*val, 40*val, -10*val, 40*val]
omega = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

def cos (x):
    res = m.cos(x)
    return res

def sin(y):
    res = m.sin(y)
    return res

def Jacobian (th):
    a, b, c, d, e, f, g = th

    J00 = (-65*((-sin(a)*cos(b)*cos(c) - sin(c)*cos(a))*cos(d) + sin(a)*sin(b)*sin(d))*cos(e) - 65*(sin(a)*sin(c)*cos(b) - cos(a)*cos(c))*sin(e))*sin(f) + (65*(sin(a)*cos(b)*cos(c) + sin(c)*cos(a))*sin(d) + 65*sin(a)*sin(b)*cos(d))*cos(f) - (-350*sin(a)*cos(b)*cos(c) - 350*sin(c)*cos(a))*sin(d) + 350*sin(a)*sin(b)*cos(d) + 400*sin(a)*sin(b) - 100*sin(a)
    J01 = (-65*(-sin(b)*cos(a)*cos(c)*cos(d) - sin(d)*cos(a)*cos(b))*cos(e) - 65*sin(b)*sin(c)*sin(e)*cos(a))*sin(f) + (65*sin(b)*sin(d)*cos(a)*cos(c) - 65*cos(a)*cos(b)*cos(d))*cos(f) + 350*sin(b)*sin(d)*cos(a)*cos(c) - 350*cos(a)*cos(b)*cos(d) - 400*cos(a)*cos(b)
    J02 = (-65*(sin(a)*sin(c) - cos(a)*cos(b)*cos(c))*sin(e) - 65*(-sin(a)*cos(c) - sin(c)*cos(a)*cos(b))*cos(d)*cos(e))*sin(f) - (-350*sin(a)*cos(c) - 350*sin(c)*cos(a)*cos(b))*sin(d) + 65*(sin(a)*cos(c) + sin(c)*cos(a)*cos(b))*sin(d)*cos(f)
    J03 = -65*(-(-sin(a)*sin(c) + cos(a)*cos(b)*cos(c))*sin(d) - sin(b)*cos(a)*cos(d))*sin(f)*cos(e) + ((65*sin(a)*sin(c) - 65*cos(a)*cos(b)*cos(c))*cos(d) + 65*sin(b)*sin(d)*cos(a))*cos(f) + (350*sin(a)*sin(c) - 350*cos(a)*cos(b)*cos(c))*cos(d) + 350*sin(b)*sin(d)*cos(a)
    J04 = (-(-65*(-sin(a)*sin(c) + cos(a)*cos(b)*cos(c))*cos(d) + 65*sin(b)*sin(d)*cos(a))*sin(e) + (65*sin(a)*cos(c) + 65*sin(c)*cos(a)*cos(b))*cos(e))*sin(f)
    J05= (-65*((-sin(a)*sin(c) + cos(a)*cos(b)*cos(c))*cos(d) - sin(b)*sin(d)*cos(a))*cos(e) - 65*(-sin(a)*cos(c) - sin(c)*cos(a)*cos(b))*sin(e))*cos(f) - (65*(sin(a)*sin(c) - cos(a)*cos(b)*cos(c))*sin(d) - 65*sin(b)*cos(a)*cos(d))*sin(f)
    J06 = 0

    J10 = (-65*((-sin(a)*sin(c) + cos(a)*cos(b)*cos(c))*cos(d) - sin(b)*sin(d)*cos(a))*cos(e) - 65*(-sin(a)*cos(c) - sin(c)*cos(a)*cos(b))*sin(e))*sin(f) + (65*(sin(a)*sin(c) - cos(a)*cos(b)*cos(c))*sin(d) - 65*sin(b)*cos(a)*cos(d))*cos(f) - (-350*sin(a)*sin(c) + 350*cos(a)*cos(b)*cos(c))*sin(d) - 350*sin(b)*cos(a)*cos(d) - 400*sin(b)*cos(a) + 100*cos(a)
    J11 = (-65*(-sin(a)*sin(b)*cos(c)*cos(d) - sin(a)*sin(d)*cos(b))*cos(e) - 65*sin(a)*sin(b)*sin(c)*sin(e))*sin(f) + (65*sin(a)*sin(b)*sin(d)*cos(c) - 65*sin(a)*cos(b)*cos(d))*cos(f) + 350*sin(a)*sin(b)*sin(d)*cos(c) - 350*sin(a)*cos(b)*cos(d) - 400*sin(a)*cos(b)
    J12 = (-65*(-sin(a)*sin(c)*cos(b) + cos(a)*cos(c))*cos(d)*cos(e) - 65*(-sin(a)*cos(b)*cos(c) - sin(c)*cos(a))*sin(e))*sin(f) - (-350*sin(a)*sin(c)*cos(b) + 350*cos(a)*cos(c))*sin(d) + 65*(sin(a)*sin(c)*cos(b) - cos(a)*cos(c))*sin(d)*cos(f)
    J13 = ((-65*sin(a)*cos(b)*cos(c) - 65*sin(c)*cos(a))*cos(d) + 65*sin(a)*sin(b)*sin(d))*cos(f) - 65*(-(sin(a)*cos(b)*cos(c) + sin(c)*cos(a))*sin(d) - sin(a)*sin(b)*cos(d))*sin(f)*cos(e) + (-350*sin(a)*cos(b)*cos(c) - 350*sin(c)*cos(a))*cos(d) + 350*sin(a)*sin(b)*sin(d)
    J14 = (-(-65*(sin(a)*cos(b)*cos(c) + sin(c)*cos(a))*cos(d) + 65*sin(a)*sin(b)*sin(d))*sin(e) + (65*sin(a)*sin(c)*cos(b) - 65*cos(a)*cos(c))*cos(e))*sin(f)
    J15 = (-65*((sin(a)*cos(b)*cos(c) + sin(c)*cos(a))*cos(d) - sin(a)*sin(b)*sin(d))*cos(e) - 65*(-sin(a)*sin(c)*cos(b) + cos(a)*cos(c))*sin(e))*cos(f) - (65*(-sin(a)*cos(b)*cos(c) - sin(c)*cos(a))*sin(d) - 65*sin(a)*sin(b)*cos(d))*sin(f)
    J16 = 0

    J20 = 0
    J21 = (-65*(-sin(b)*sin(d) + cos(b)*cos(c)*cos(d))*cos(e) + 65*sin(c)*sin(e)*cos(b))*sin(f) + (-65*sin(b)*cos(d) - 65*sin(d)*cos(b)*cos(c))*cos(f) - 350*sin(b)*cos(d) - 400*sin(b) - 350*sin(d)*cos(b)*cos(c)
    J22 = (65*sin(b)*sin(c)*cos(d)*cos(e) + 65*sin(b)*sin(e)*cos(c))*sin(f) + 65*sin(b)*sin(c)*sin(d)*cos(f) + 350*sin(b)*sin(c)*sin(d)
    J23 =  -65*(-sin(b)*sin(d)*cos(c) + cos(b)*cos(d))*sin(f)*cos(e) + (-65*sin(b)*cos(c)*cos(d) - 65*sin(d)*cos(b))*cos(f) - 350*sin(b)*cos(c)*cos(d) - 350*sin(d)*cos(b)
    J24 = (-(-65*sin(b)*cos(c)*cos(d) - 65*sin(d)*cos(b))*sin(e) + 65*sin(b)*sin(c)*cos(e))*sin(f)
    J25 = (-65*(sin(b)*cos(c)*cos(d) + sin(d)*cos(b))*cos(e) + 65*sin(b)*sin(c)*sin(e))*cos(f) - (-65*sin(b)*sin(d)*cos(c) + 65*cos(b)*cos(d))*sin(f)
    J26 = 0

    return np.array([[J00, J01, J02, J03, J04, J05, J06],
                     [J10, J11, J12, J13, J14, J15, J16],
                     [J20, J21, J22, J23, J24, J25, J26]])


des_vel = np.array([[ 0.000000, -0.000000, 0.000000],
                   [0,-0.007737,1.063205],
                   [0,-0.205197,3.811516],
                   [0,-1.274944,7.553738],
                   [0,-4.293985,11.273639],
                   [0,-10.031497,13.184624],
                   [0,-17.75871,10.793729],
                   [0,-24.283745,2.269418],
                   [0,-24.742757,-11.160556],
                   [0,-15.943179,-24.06408],
                   [0,0,-29.452431],
                   [0,15.943179,-24.06408],
                   [0,24.742757,-11.160556],
                   [0,24.283745,2.269418],
                   [0,17.75871,10.793729],
                   [0,10.031497,13.184624],
                   [0,4.293985,11.273639],
                   [0,1.274944,7.553738],
                   [0,0.205197,3.811516],
                   [0,0.007737,1.063205]]
                   )

i = 0
j = 0
dt = 1

"""while (i<20):
    J = Jacobian(th)
    Jpseudo = np.linalg.pinv(J)
    omega = np.dot(Jpseudo,des_vel[i])
    th = dt*omega + th
    while (j<7):
        print(th[j]*cons, end=' ')
        j=j+1
    print(end='\n')
    j=0
    i = i+1"""

th = [10*val, 15*val, -10*val, -100*val, 40*val, -10*val, 40*val]
vel = [0, -24.742757, -11.160556]

J = Jacobian(th)
Jpseudo = np.linalg.pinv(J)
omega = np.dot(Jpseudo,vel)

while(i<7):
    print(omega[i]*cons)
    i+=1

