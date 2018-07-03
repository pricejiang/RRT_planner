from scipy.integrate import odeint
import numpy as np
import matplotlib.pyplot as plt
import math


'''
    Bicycle Car Dynamic model. 
    State represetned by [xg, yg, theta, vy, r], where: 
        xg, yg are the location center of gravity of the car
        theta is orientation of the car
        vy is the lateral speed
        r is the yaw rate/angular velocity

        longitudinal speed vx is viewed as a constant in this model
    Source: http://www.cs.cmu.edu/~motionplanning/reading/PlanningforDynamicVeh-1.pdf 
    Inputs: Xn - current state; a list
            delta_f - steering angle of the car
    Output: dydt
'''
def car_dynamic(Xn, delta_f):
    # xg, yg represent the location center of gravity of the car
    # theta is orientation
    # vy is the lateral speed
    # r is the yaw rate/angular velocity
    xg, yg, theta, vy, r = Xn
    xg = float(xg)
    yg = float(yg)
    theta = float(theta)
    vy = float(vy)
    r = float(r)

    # Some constants; NOTE: subject to variation if a better configuration is found
    # mass of the car, unit: kg
    m = 1700
    # longitudinal velocity, unit: m/s
    vx = 28
    # length of front half of the car, unit: m
    Lf = 2
    # length of rear half of the car, unit: m
    Lr = 2
    # cornering stiffness coefficient
    Cf = 20000
    Cr = 20000

    # moment of inertia
    Iz = 6000

    cosTheta = np.cos(theta)
    sinTheta = np.sin(theta)
    cosDelta = np.cos(delta_f)

    # Constants A,B,C,D,E,F given in the paper
    A = -(Cf*cosDelta + Cr)/(m*vx)
    B = (-Lf*Cf*cosDelta + Lr*Cr)/(m*vx) - vx
    C = (-Lf*Cf*cosDelta + Lr*Cr)/(Iz*vx)
    D = -(Lf*Lf*Cf*cosDelta + Lr*Lr*Cr)/(Iz*vx)
    E = (Cf*cosDelta)/m
    F = (Lf*Cf*cosDelta)/Iz

    # differential equations
    xg_dot = vx*cosTheta - vy*sinTheta
    yg_dot = vx*sinTheta + vy*cosTheta
    theta_dot = r
    vy_dot = A*vy + B*r + E*delta_f
    r_dot = C*vy + D*r + F*delta_f
    
    # Return dy/dt
    dydt = np.array([float(xg_dot), float(yg_dot), float(theta_dot), float(vy_dot), float(r_dot)])
    return dydt 

'''
    This simulator is for test only 
    NOTE: need to add 't' argument in the above car_dynamic function to perform test
'''
def TC_Simulate(Mode, initial, time_bound):
    time_step = 0.05
    time_bound = float(time_bound)
    initial = [float(tmp)  for tmp in initial]
    number_points = int(np.ceil(time_bound/time_step))
    t = [i*time_step for i in range(0,number_points)]
    if t[-1] != time_step:
		t.append(time_bound)

    newt = [] 
    for step in t:
        newt.append(float(format(step, '.2f')))
    t = newt
    delta = 0
    sol = odeint(car_dynamic, initial, t, args=(delta,), hmax=time_step)

    # Construct the final output
    trace = []
    for j in range(len(t)):
        tmp = []
        tmp.append(t[j])
        tmp.append(float(sol[j, 0]))
        tmp.append(float(sol[j, 1]))
        tmp.append(float(sol[j, 2]))
        tmp.append(float(sol[j, 3]))
        tmp.append(float(sol[j, 4]))
        trace.append(tmp)
    return trace        

if __name__ == "__main__":
	sol = TC_Simulate('Default', [5.0, 5.0, 0, 10, 0], 20)
        for s in sol:
            print s
        a = [row[1] for row in sol]
        b = [row[2] for row in sol]
    
        plt.plot(a, b, '-r')
        plt.show()
