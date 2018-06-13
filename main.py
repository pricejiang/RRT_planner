from RRT import *
import random
from z3 import *
from pydraw import *

def getObs(n):
    obs = []
    # obs.append(((50, 0), (50, 250)))
    obs.append(((100, 250), (100, 500)))
    # obs.append(((150, 0), (150, 250)))
    obs.append(((200, 250), (200, 500)))
    # obs.append(((250, 0), (250, 250)))
    obs.append(((300, 250), (300, 500)))
    # obs.append(((350, 0), (350, 250)))
    obs.append(((400, 250), (400, 500)))
    # obs.append(((450, 0), (450, 250)))
    return obs

def main():
    G = RRT([350, 270, 0.0, 0.0, 0.0])
    obs = []
    
    n = 1
    obs = getObs(n)

    screen = initDraw()
    path = G.plan(5000, (450,500,450,500), 0.1, obs, 0, screen)
    if path == None:
        print 'No path find'
    else:
        print 'goal reached'
        print path
    
if __name__ == '__main__':
    main()