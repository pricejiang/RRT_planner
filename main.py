from RRT import *
import random
from z3 import *
from pydraw import *
import time

'''
    This function gives the configuration of the obstacles in the space. 
    The obstacles are line segments defined by two endpoints. 
'''
def getObs(n):
    obs = []
    obs.append(((50, 0), (50, 250)))
    obs.append(((100, 250), (100, 500)))
    obs.append(((150, 0), (150, 250)))
    obs.append(((200, 250), (200, 500)))
    obs.append(((250, 0), (250, 250)))
    obs.append(((300, 250), (300, 500)))
    obs.append(((350, 0), (350, 250)))
    obs.append(((400, 250), (400, 500)))
    obs.append(((450, 0), (450, 250)))
    return obs

def main():
    # Screen window size
    winsize = [500,500]
    # Initialize RRT 
    G = RRT([270, 200, 6.0, 0.0, 0.0])

    # Obtain obstacle and goal
    obs = []
    n = 1
    obs = getObs(n)
    goal = (450,500,450,500)

    # Initialize screen
    screen = initDraw(winsize)

    # Call planner
    startTime = time.time()
    ret = G.plan(5000, goal, 0.2, obs, screen, winsize)
    endTime = time.time()

    # Parse and draw  result
    if ret == None:
        print 'No path find'
    elif type(ret) == list:
        print 'goal reached'
        print ret
        drawPath(screen, ret, goal, obs)
    else:
        for b in ret:
            drawRec(screen, ret[b])

    print 'Used time ', endTime-startTime, ' seconds'
    
if __name__ == '__main__':
    main()