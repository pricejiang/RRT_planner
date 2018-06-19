from RRT import *
import random
from z3 import *
from pydraw import *
import time

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
    G = RRT([370, 200, 6.0, 0.0, 0.0])
    obs = []
    
    n = 1
    obs = getObs(n)
    goal = (450,500,450,500)

    screen = initDraw()

    startTime = time.time()
    ret = G.plan(5000, goal, 0.1, obs, 1, screen)
    endTime = time.time()

    if ret == None:
        print 'No path find'
    elif ret == list:
        print 'goal reached'
        print ret
        drawPath(screen, ret, goal, obs)
    else:
        drawRec(screen, ret)

    print 'Used time ', endTime-startTime, ' seconds'
    
if __name__ == '__main__':
    main()