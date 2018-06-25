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

