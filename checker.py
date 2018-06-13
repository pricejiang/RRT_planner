from z3 import *
from math import sqrt, acos, sin

'''
    This function returns the equation of the line defined by the two points given
'''
def getEqn(x, y, start, end):
        x1 = start[0]
        y1 = start[1]

        x2 = end[0]
        y2 = end[1]

        # If the line is not perpendicular to x-axis, calculate slope
        if x1 != x2:
            k = (y1-y2)/float(x1-x2)
        # Otherwise, we cannot calculate the slope
        else:
            return x == x1

        return y == k*(x-x2) + y2
        
'''
    This function checks if Xnew is within the goal region
'''
def goalCheck(Xnew, goal):
    if(Xnew[0] >= goal[0] and Xnew[0] <= goal[1] and Xnew[1] >= goal[2] and Xnew[1] <= goal[3]):
        return True
    else: 
        return False

'''
    This functions checks if rrtLine intersects with a single obsLine
    Return: True if intersects
            False if not 
'''
def obsCheck(x, y, rrtLine, obsLine, p1, p2, ob1, ob2):
    # Initialize solver() and add two lines into it
    s = Solver()
    s.add(rrtLine)
    s.add(obsLine)

    # Add endpoints to define boundry
    if p1[0] < p2[0]:
        s.add(And(x>=p1[0], x<=p2[0]))
    else:
        s.add(And(x>=p2[0], x<=p1[0]))
    
    if p1[1] < p2[1]:
        s.add(And(y>=p1[1], y<=p2[1]))
    else:
        s.add(And(y>=p2[1], y<=p1[1]))
    
    s.add(And(x>=ob1[0], x<=ob1[0]))
    s.add(And(y>=ob1[1], y<=ob2[1]))

    # Check result
    if s.check() == sat:
        return True
    else:
        return False

'''
    This function checks whether the line defined by p1 and p2 intersects with any of the obstacle line
    Return: True if intersects
            False if not
'''
def collisionCheck(p1, p2, obs):
    x1 = p1[0]
    y1 = p1[1]

    x2 = p2[0]
    y2 = p2[1]

    x = Real('x')
    y = Real('y')

    flag = False
    rrtLine = getEqn(x, y, (x1,y1), (x2, y2))
    # Iterates to see if rrtLine intersects with any obstacles 
    for ob in obs:
        obsLine = getEqn(x, y, ob[0], ob[1])
        if obsCheck(x, y, rrtLine, obsLine, (x1,y1), (x2, y2), ob[0], ob[1]) == True:
            flag = True
    
    return flag

def eucDistance(p1, p2):
    return sqrt((p1[0]-p2[0])*(p1[0]-p2[0]) + (p1[1]-p2[1])*(p1[1]-p2[1]))

def randomChecker(p, ob):
    l1 = eucDistance(p, ob[0])
    l2 = eucDistance(p, ob[1])
    L = eucDistance(ob[0], ob[1])

    beta = acos((l2*l2 + L*L - l1*l1)/(2*l2*L))
    h = sin(beta)*l2
    return h < 10

'''
    This function checks if the given point 
    can connect directly to the goal region (center of the square)
'''
def connectChecker(p, goal, obs):
    a = ((goal[1]+goal[0])/2, (goal[3]+goal[2])/2)
    # b = (goal[0], goal[3])
    # c = (goal[1], goal[2])
    # d = (goal[1], goal[3])
    return not collisionCheck(p, a, obs)
 


if __name__ == '__main__':
    # print randomChecker((1,0), ((16,-1), (16,3)))

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
    print connectChecker((420, 200), (450,500,450,500), obs)
