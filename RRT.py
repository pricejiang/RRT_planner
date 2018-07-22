import numpy as np
# from Car_Dynamic import *
import random
from pydraw import *
from z3 import *
from checker import *
import pdb
from math import atan, ceil
import time
from operator import itemgetter
from box import *
from helper import *


'''
    node class: basic element of RRT search tree
'''
class node():
    def __init__(self, state, parent):
        self.state = state
        self.parent = parent
        self.children = []



'''
    RRT class: the RRT planner class combined with the bicycle car model
'''
class RRT():
    
    def __init__(self, init, selectInput, randomConfig, tryInput, winsize):
        self.nodes = [] # data structure to hold tree nodes
        self.Xinit = node(init, None)   # initial point
        self.nodes.append(self.Xinit)   
        self.Xnear = node(None, None)   # Xnear node 
        # Functions introduced from modules
        self.selectInput = selectInput  
        self.randomConfig = randomConfig
        self.tryInput = tryInput

        self.winsize = winsize


    '''
        findNearest: find the nearest node in RRT search tree to the point Xrand 
        Input: Xrand - a random point on the space Xfree
        Output: the nearest node in RRT search tree to the point Xrand 
    '''
    def findNearest(self, Xrand):
        nn = self.nodes[0]
        for p in self.nodes:
            # Avoid picking the same node as last time
            if p == self.Xnear:
                continue
            
            if dist(p.state, Xrand) < dist(nn.state, Xrand):
                nn = p
        return nn

    '''
        This function returns a path from Xinit to Xnew
        Inputs: Xnew - the endpoint of the path we want to calculate
        Output: a path from Xinit to Xnew
    '''
    def getPath(self, Xnew):
        node = Xnew
        path = []
        path.append(node.state)
        # Iterate to find the inital point
        while(node != self.Xinit):
            node = node.parent
            path.append(node.state)
        path.reverse()
        return path
   
    '''
        This function is the RRT planner. It uses a modified RRT algorithm to 
        find the path from Xinit to Xgoal region. 

        Inputs: K - total number of nodes/iterations allowed
                goal - goal region
                p - try_goal_probability; the probability that Xgoal is picked as Xrand
                obs - obstacle region Xobs
                screen - the pygame screen for printing the game status
                data - important data from input file
        Output: the path from Xinit to Xgoal

    '''
    def plan(self, K, goal, obs, screen, data):
        p, flag, k, epsilon = self.parseData(data)
        if p > 1 or p < 0:
            print "Invalid goalbias p; please set p = [0.0, 1.0]"
            return None
        color = (155, 240, 200)
        region = []
        degree = len(self.Xinit.state)
        for i in range(K):
            print i, 'th iteration'

            # Set Xrand
            if random.random() < p:
                Xrand = ((goal[1]+goal[0])/2, (goal[3]+goal[2])/2, random.random()*2*np.pi, 0, 0)
            else:
                Xrand = self.randomConfig(self.winsize[0], self.winsize[1])

            # pdb.set_trace()
            # drawNode(screen, Xrand, 1)
            # Pick Xnear
            self.Xnear = self.findNearest(Xrand)
            print self.Xnear.state
            # drawNode(screen, self.Xnear.state, 2)

            # Calculate new state from using Xrand and Xnear
            u = self.selectInput(Xrand, self.Xnear.state, obs)
            if u == None:
                continue

            # Check if the new state is within the hopeless region
            if region != []:
                if regionChecker(region, u):
                    continue

            # Check if connecting to Xnew will collide with obstacles
            p1 = (int(self.Xnear.state[0]), int(self.Xnear.state[1]))
            p2 = (int(u[0]), int(u[1]))
            collision = collisionChecker(p1, p2, obs)

            if i == 0 and collision:
                print "Initial node on obstacle!! Please pick another initial node. "
                return None
            # If no intersections with obstacles, add Xnew to the tree and upate the graph
            if not collision:
                Xnew = node(u, self.Xnear)
                self.nodes.append(Xnew)
                self.Xnear.children.append(Xnew)
                
                drawScreen(screen, p1, p2, goal, obs, color)
            # Else, delete a few nodes on the branch
            elif flag == 1:
                print "Collide with Obstacles"
                # Obtain X_array and epsilon_array
                X_array = self.collisionClean(self.Xnear, k)
                epsilon_array = self.branchElim(X_array, epsilon)
                boxes = []
                for i in range(len(X_array)):
                    b = box(X_array[i], epsilon_array[i])
                    boxes.append(b)

                # Draw the reachtube boxes
                drawRec(screen, boxes, obs, color)
                # Perform a few checks to get the corner points for subtree initiation
                Bk = boxes[-1] # last box
                boxCheck =  boxChecker(Bk, obs, self.winsize)
                cornerPoints = self.getXsubInit(boxCheck, Bk)
                

                # Grow the subtree with cornerPoints
                # Set the new node's direction to goal or to the closed node Xc; it might not be the best choice
                for point in cornerPoints:
                    point = [int(point[0]), int(point[1])]
                   
                    Xc = self.findNearest(point)
                    # Determine the angle to start 
                    # If the point can be directly connected to goal, set theta directly to the goal
                    if connectChecker(point, goal, obs):
                        gc = ((goal[1]+goal[0])/2, (goal[3]+goal[2])/2)
                        theta_prime = atan((gc[1]-point[1])/(gc[0]-point[0]))
                    # Otherwise, set to the closed node Xc in the tree
                    else:
                        theta_prime = Xc.state[2]
                    point.append(theta_prime)

                    # Set other degrees of parameters to closed node Xc in the tree
                    for j in range(degree):
                        if j < 3:
                            continue
                        point.append(Xc.state[j])
                    Xnew = node(point, Xc)
                    self.nodes.append(Xnew)
                # Put the outer most box just generted into region list
                region.append(parseBox(boxes))

            
            # If reaches the goal region, find the path and return it
            if goalChecker(Xnew.state, goal):
                return self.getPath(Xnew)
        
        return None


    '''
        OLD: 
        This functions discard some nodes in self.nodes when collision happens
        The reason to do this is because once a collision happens, 
        this tree branch will be useless

        NEW: 
        This function returns a list of nodes on the branch that collides 
        with obstacles. 

        Inputs: Xnew - the point where collision happens
        Output: A list of nodes on the collision branch
    '''
    def collisionClean(self, Xn, k):
        x = Xn
        X_array = []
        # pdb.set_trace()
        for i in range(k):
            if x == self.Xinit:
                break
            X_array.append(x)
            if x in self.nodes:
                self.nodes.remove(x)
            x = x.parent
        X_array.reverse()
        return X_array
        
    '''
        This function finds a series of boxes with size epsilon
        centered by nodes on a collision branch. 

        Inputs: X_array - a list of nodes on the collsion branch
                epsilon - the radius of the initial box
        Output: epsilon_array - the radius of all boxes 
        NOTE:   This function is subject to changes. Use under-approximation instead of 
                over-approximation that is currently in use
    '''
    def branchElim(self, X_array, epsilon):
        X0 = X_array[0]
        X0_array = []
        epsilon_array = []
        epsilon_array.append(epsilon)
        # Randomly choosing 16 points in the box B0 defined by B(X0, epsilon)
        for i in range(16):
            # The first point is X0
            if i == 0:
                Xn = X0.state
            # Random configuration on all dimensions
            else:
                x = random.randint(int(X0.state[0])-epsilon, int(X0.state[0])+epsilon)
                if x > self.winsize[0]:
                    x = self.winsize[0]
                if x < 0:
                    x = 0
                y = random.randint(int(X0.state[1])-epsilon, int(X0.state[1])+epsilon)
                if y > self.winsize[1]:
                    y = self.winsize[1]
                if y < 0:
                    y = 0
                Xn = [x,y]

                for i in range(len(X0.state)):
                    if i < 2:
                        continue
                    tmp = random.randint(int(X0.state[i])-epsilon, int(X0.state[i])+epsilon)
                    Xn.append(tmp)
            X0_array.append(Xn)
        epsilon_prime = epsilon
        Xc = X0
        # For all nodes in the X_array list
        for i in range(len(X_array)):
            # First, get all 4 extreme points of box
            lt, rt, lb, rb = self.getExtreme(Xc, epsilon_prime)
            X0_array.append(lt)
            X0_array.append(rt)
            X0_array.append(lb)
            X0_array.append(rb)
            X0_array.reverse()
            # Then, get new epsilon' and X1_array with nodes in the new box 
            # generated by random 20 nodes in current box with all possible inputs
            epsilon_prime, X1_array = self.pi_transition(X0_array[:20], epsilon_prime)
            epsilon_prime = int(epsilon_prime)
            epsilon_array.append(epsilon_prime)
            X0_array = X1_array
            # Shuffle the list
            random.shuffle(X0_array)
            Xt = X0_array[i]
            Xc = node(Xt, None)
        epsilon_array.pop(-1)
        return epsilon_array

    '''
        This function calculates the Bn+1 with Bn box defined by (Xn, epsilon)
        where Xn is a list of nodes, and epsilon is the radius of Bn

        Inputs: X0_array - a list of the nodes in the box Bn
                epsilon - the radius of the box Bn
        Output: epsilon_prime - the radius of the box Bn+1
                X1_array - a list of the nodes in the box Bn+1
    '''
    def pi_transition(self, X0_array, epsilon):
        X1_array = []
        for Xn in X0_array:
            # Try out all the possible input at a given node Xn
            # the output will form a box 
            output = self.tryInput(Xn)
            X1_array = X1_array + output

        X1_array = sorted(X1_array, key=itemgetter(1))
        h = X1_array[-1][1] - X1_array[0][1] # Get the height of the box
        X1_array = sorted(X1_array, key=itemgetter(0))
        w = X1_array[-1][0] - X1_array[0][0] # Get the width of the box

        epsilon_prime = ceil((h+w)/2)/2 # Obtain epsilon_prime by average height and width
        return epsilon_prime, X1_array
    
    
    '''
        This function gets the all 4 corner extreme points of box Bn defined by B(X0, epsilon)
        Inputs: X0 - a node in the space; center of Bn
                epsilon - radius of Bn
        Output: lt - left-top point
                rt - right-top point
                lb - left-bottom point
                rb - right-bottom point
    '''
    def getExtreme(self, X0, epsilon):
        x = X0.state[0]
        y = X0.state[1]
        n = len(X0.state)

        # Calculate lt
        x1 = x - epsilon
        y1 = y - epsilon
        lt = [x1, y1]
        for i in range(n):
            if i < 2:
                continue
            tmp = random.randint(int(X0.state[i])-epsilon, int(X0.state[i])+epsilon)
            lt.append(tmp)

        # Calculate rt
        x2 = x + epsilon
        y2 = y - epsilon
        rt = [x2, y2]
        for i in range(n):
            if i < 2:
                continue
            tmp = random.randint(int(X0.state[i])-epsilon, int(X0.state[i])+epsilon)
            rt.append(tmp)

        # Calculate lb
        x3 = x - epsilon
        y3 = y + epsilon
        lb = [x3, y3]
        for i in range(n):
            if i < 2:
                continue
            tmp = random.randint(int(X0.state[i])-epsilon, int(X0.state[i])+epsilon)
            lb.append(tmp)

        # Calculate rb
        x4 = x + epsilon
        y4 = y + epsilon
        rb = [x4, y4]
        for i in range(n):
            if i < 2:
                continue
            tmp = random.randint(int(X0.state[i])-epsilon, int(X0.state[i])+epsilon)
            rb.append(tmp)

        return lt, rt, lb, rb
    
    '''
        This function returns a corner of the outer most box as the subtree initial node
    '''
    def getXsubInit(self, boxCheck, Bk):
        # Four corner points coordinate
        lt = Bk.lt
        rt = Bk.rt
        lb = Bk.lb
        rb = Bk.rb
        # Randomly select a corner 
        ret = []
        if not boxCheck[0]:
            ret.append(lt)
        if not boxCheck[1]:
            ret.append(rt)
        if not boxCheck[2]:
            ret.append(lb)
        if not boxCheck[3]:
            ret.append(rb)
        
        random.shuffle(ret)
        return ret

    def parseData(self, data):
        p = data["goalBias"]
        flag = data["method"]
        k = data["kvalue"]
        epsilon = data["epsilon"]
        return p, flag, k, epsilon
