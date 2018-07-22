from math import sqrt, acos, sin
import numpy as np

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
def goalChecker(Xnew, goal):
    if(Xnew[0] >= goal[0] and Xnew[0] <= goal[1] and Xnew[1] >= goal[2] and Xnew[1] <= goal[3]):
        return True
    else: 
        return False

'''
    This functions checks if a line intersects with a single obsLine
    Return: True if intersects
            False if not 
'''

def obsCheck(a,b,c,d):
    a1 = b[1]-a[1]
    b1 = a[0]-b[0]
    c1 = a1*a[0]+b1*a[1]

    a2 = d[1]-c[1]
    b2 = c[0]-d[0]
    c2 = a2*c[0]+b2*c[1]

    determinant = a1*b2 - a2*b1

    if determinant == 0:
        return False
    else:
        x = (b2*c1 - b1*c2)/determinant
        y = (a1*c2 - a2*c1)/determinant
        if x - min(a[0], b[0]) < -0.01 or x - max(a[0], b[0]) > 0.01:
            return False
        if x - min(c[0], d[0]) < -0.01 or x - max(c[0], d[0]) > 0.01:
            return False
        if y - min(a[1], b[1]) < -0.01 or y - max(a[1], b[1]) > 0.01:
            return False
        if y - min(c[1], d[1]) < -0.01 or y - max(c[1], d[1]) > 0.01:
            return False
        return True
'''
    This function checks whether the line defined by p1 and p2 intersects with any of the obstacle line
    Return: True if intersects
            False if not
'''
def collisionChecker(p1, p2, obs):
    flag = False
    # Iterates to see if rrtLine intersects with any obstacles 
    for ob in obs:
        if obsCheck(p1,p2,ob[0],ob[1])==True:
            flag = True
    return flag

'''
    This function identifies the safe corners of the box Bk. 
    By safe I mean the corner is directly connectable from its center (xg, yg). 
    NOTE However, this way might not be the correct way to do it, 
    since directly connectable does not mean approachable by the dynamics. 

    Inputs: Bk - the box to deal with
            obs - the list of obstacles
    Output: a,b,c,d - booleans represents the safety of corners left-top, right-top, left-bottom, right-bottom
'''
def boxChecker(Bk, obs, winsize):
    xg = Bk.xg
    yg = Bk.yg
    r = Bk.epsilon
    # Four corner points coordinate
    lt = Bk.lt
    rt = Bk.rt
    lb = Bk.lb
    rb = Bk.rb

    print 'lt, rt, lb, rb are ', lt, rt, lb, rb
    print 'xg, yg', (xg, yg)

    a,b,c,d = False, False, False, False

    for ob in obs:
        if obsCheck(lt, (xg,yg), ob[0], ob[1]):
            a = True
        if obsCheck(rt, (xg,yg), ob[0], ob[1]):
            b = True
        if obsCheck(lb, (xg,yg), ob[0], ob[1]):
            c = True
        if obsCheck(rb, (xg,yg), ob[0], ob[1]):
            d = True
    
    return a,b,c,d


'''
    This function checks whether the given point p 
    can be conncected to the goal directly
'''
def connectChecker(p, goal, obs):
    a = ((goal[1]+goal[0])/2, (goal[3]+goal[2])/2)

    return not collisionChecker(p, a, obs)

def regionChecker(region, X):
    xg = X[0]
    yg = X[1]
    theta = X[2]
    for r in region:
        box = r[0]
        theta_range = r[1]
        if xg <= box.right and xg >= box.left and yg <= box.bottom and yg >= box.top:
            if theta <= theta_range[1]+np.pi/9 and theta >= theta_range[0]-np.pi/9:
                return True

    return False
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
