import numpy as np
from Car_Dynamic import *
import random
from pydraw import *
from z3 import *
from checker import *
# import pygame, sys
# from pygame.locals import *

delta_t = 0.2
min_steer = -np.pi/3
max_steer = np.pi/3
height = 500
width = 500

'''
    node class: basic element of RRT search tree
'''
class node():
    def __init__(self, state, parent):
        self.state = state
        self.parent = parent
        self.children = []
        self.Xcritic = None

'''
    RRT class: the RRT planner class combined with the bicycle car model
'''
class RRT():
    
    def __init__(self, init):
        self.nodes = []
        self.Xinit = node(init, None)
        self.nodes.append(self.Xinit)

    '''
        randomConfig: this function generates a random point on the space Xfree
        Input: obs - obstacle region
        Output: a random point on the space Xfree
    '''
    def randomConfig(self, obs):
        while True:
            x = random.random()*width
            y = random.random()*height
            theta = random.random()*2*np.pi
            
            tmp = False
            for ob in obs:
                if randomChecker((x,y), ob):
                    tmp = True
            # print tmp   
            if not tmp:
                break
       
        return (x, y, theta, 0, 0)

    '''
        findNearest: find the nearest node in RRT search tree to the point Xrand 
        Input: Xrand - a random point on the space Xfree
        Output: the nearest node in RRT search tree to the point Xrand 
    '''
    def findNearest(self, Xrand, flag):
        nn = self.nodes[0]
        if flag == 2:
            flag = 3
            print 'return flag'
            return (self.Xcritic, flag)
        for p in self.nodes:
            if self.dist(p.state, Xrand) < self.dist(nn.state, Xrand):
                nn = p
        return (nn, flag)

    def newState(self, Xn, delta_f):
        # Fourth-Order Runge-Kutta method
        k1 = car_dynamic(Xn, delta_f)
        k2 = car_dynamic(Xn+k1/2, delta_f)
        k3 = car_dynamic(Xn+k2/2, delta_f)
        k4 = car_dynamic(Xn+k3, delta_f)

        Xnew = Xn + (k1 + 2*k2 + 2*k3 + k4)*delta_t/6
        return Xnew

    def selectInput(self, Xrand, Xnear):
        delta_f = min_steer
        bestState = self.newState(Xnear, delta_f)
        bestDistance = float(self.dist(bestState, Xrand))

        while delta_f < max_steer:
            Xnew = self.newState(Xnear, delta_f)
            distance = self.dist(Xnew, Xrand)
            if  distance < bestDistance:
                bestState = Xnew
                bestDistance = distance
            delta_f += np.pi/60 # increment of approximately 3 degrees per iteration
        
        return bestState
    
  
    # def extend(self, Xrand):
    #     Xnear = self.findNearetst(Xrand)
    #     res = self.selectInput(Xrand, Xnear.state)
    #     Xnew = node(res, Xnear)
    #     return Xnear, Xnew
    
    def getPath(self, Xnew):
        node = Xnew
        path = []
        path.append(node)
        while(node != self.Xinit):
            node = node.parent
            path.append(node)
        path.reverse()
        return path
   

    def plan(self, K, goal, p, obs):
        if p > 1 or p < 0:
            print "invalid p"
            return None
        screen = initDraw()
        flag = 0
        for i in range(K):
            if random.random() < p:
                if flag != 2 and flag != 3:
                    flag = 1
                Xrand = (((goal[1]+goal[0])/2, (goal[3]+goal[2])/2, random.random()*2*np.pi, 0, 0))
                # print Xrand
            else:
                flag = 0
                Xrand = self.randomConfig(obs)
            
            # Xnear, Xnew = self.extend(Xrand, flag)
            
            Xnear, flag = self.findNearest(Xrand, flag)
            print flag
            # if flag == 1:
            if connectChecker(Xnear.state, goal, obs) and flag != 3:
                print 'critic find'
                self.Xcritic = Xnear
                flag = 2
                p = 1
                
                
            res = self.selectInput(Xrand, Xnear.state)
            Xnew = node(res, Xnear)
            # print Xnew.state
            # print flag

            p1 = (int(Xnear.state[0]), int(Xnear.state[1]))
            p2 = (int(Xnew.state[0]), int(Xnew.state[1]))

            # print "start is", p1
            # print "end is ", p2

            # If no intersections with obstacles, add Xnew to the tree and upate the graph
            if not collisionCheck(p1, p2, obs):
                self.nodes.append(Xnew)
                Xnear.children.append(Xnew)
                drawScreen(screen, p1, p2, goal, obs)
            # New Add-in
            # Else, delete a few nodes on the branch
            else:
                print "clean"
                self.collisionClean(Xnew)

            # self.nodes.append(Xnew)
            # drawScreen(screen, p1, p2, goal, obs)
            # print getEqn(Real('x'), Real('y'), p1, p2)
            
            if goalCheck(Xnew.state, goal):
                return self.getPath(Xnew)
        
        return None
        

    # Helper Functions
    def dist(self, s1, s2):
        x1 = s1[0]
        y1 = s1[1]
        theta1 = s1[2]

        x2 = s2[0]
        y2 = s2[1]
        theta2 = s2[2]
        
        theta_diff = min(theta1-theta2, 2*np.pi - (theta1-theta2))
        return np.sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + theta_diff*theta_diff)

    def collisionClean(self, Xnew):
        n = Xnew.parent
        while True:
            if n == self.Xinit:
                break
            if len(n.children) >= 2:
                break
            self.nodes.remove(n)
            n = n.parent


