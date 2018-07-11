from RRT import *
import random
from z3 import *
from pydraw import *
import time
from util import *

'''
    This function gives the configuration of the obstacles in the space. 
    The obstacles are line segments defined by two endpoints. 
'''
def getObs(n):
    print n
    obs = []
    if n == 1:
        obs.append(((50, 0), (50, 250)))
        obs.append(((100, 250), (100, 500)))
        obs.append(((150, 0), (150, 250)))
        obs.append(((200, 250), (200, 500)))
        obs.append(((250, 0), (250, 250)))
        obs.append(((300, 250), (300, 500)))
        obs.append(((350, 0), (350, 250)))
        obs.append(((400, 250), (400, 500)))
        obs.append(((450, 0), (450, 250)))
        obs.append(((0, 0), (0, 500)))
        obs.append(((0, 0), (500, 0)))
        obs.append(((0, 500), (500, 500)))
        obs.append(((500, 0), (500, 500)))
    elif n == 2:
        obs.append(((0, 150), (250, 150)))
        obs.append(((250, 250), (500, 250)))
        obs.append(((0, 350), (250, 350)))
        obs.append(((0, 0), (0, 500)))
        obs.append(((0, 0), (500, 0)))
        obs.append(((0, 500), (500, 500)))
        obs.append(((500, 0), (500, 500)))
    else:
        obs.append(((0, 0), (0, 500)))
        obs.append(((0, 0), (500, 0)))
        obs.append(((0, 500), (500, 500)))
        obs.append(((500, 0), (500, 500)))
    return obs

def main(argv):
    if len(argv) < 2:
        print "Must add a model name"
        return None

    module, data = parseInput(argv)
    selectInput, randomConfig, tryInput = module
    initial = data["initial"]
    # Screen window size
    winsize = [500,500]
    # Initialize RRT 
    G = RRT(initial, selectInput, randomConfig, tryInput, winsize)

    # Obtain obstacle and goal
    obs = []
    # n = raw_input('Please select obstacle scenario, 1 or 2\n')
    n = 2
    obs = getObs(int(n))
    goal = (450,500,450,500)
    # Initialize screen
    screen = initDraw(winsize)

    # Call planner
    startTime = time.time()
    ret = G.plan(5000, goal, 0.3, obs, screen, data["method"])
    endTime = time.time()

    # Parse and draw result
    if ret == None:
        print 'No path find'
    elif type(ret) == list:
        print 'goal reached'
        print ret
        drawPath(screen, ret, goal, obs)

    print 'Used time ', endTime-startTime, ' seconds'
    
if __name__ == '__main__':
    main(sys.argv)