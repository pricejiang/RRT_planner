import pygame, sys
from pygame.locals import *
from random import randint
from checker import boxChecker

white = (255, 240, 200)
black = (20, 20, 40)
red = (255, 0, 0)
green = (0, 255, 0)
blue = (0, 0, 255)

'''
    This function nitializes pygame interface
'''
def initDraw(winsize):
    pygame.init()
    screen = pygame.display.set_mode(winsize)
    pygame.display.set_caption('Bicycle Model       Test        05/2018')
    
    screen.fill(black)
    return screen

'''
    This function updates current status of planner
    Inputs: screen - the screen to draw on
            p1 - endpoint of the line segment
            p2 - the other endpoint
            goal - goal region
            obs - obstacle region
'''
def drawScreen(screen, p1, p2, goal, obs, color):
    pygame.draw.rect(screen, red, (goal[0],goal[2],goal[1]-goal[0],goal[3]-goal[2]))
    pygame.draw.line(screen,color,p1,p2)
    for ob in obs:
        pygame.draw.line(screen, blue, ob[0], ob[1])
    pygame.display.update()

    drawEnd()

'''
    This functions draws the path returned by the planner
    Inputs: screen - the screen to draw on 
            path - the path returned by planner
            goal - goal region
            obs - obstacle region
'''
def drawPath(screen, path, goal, obs):
    # screen.fill(black)
    for i in range(len(path)):
        if i == len(path)-1:
            break
        p1 = (path[i][0], path[i][1])
        p2 = (path[i+1][0], path[i+1][1])
        drawScreen(screen, p1, p2, goal, obs, white)
    
    wait = raw_input('---------- PAUSED, press ENTER to continue ----------')

'''
    This function draws the a serie of boxes defined by Xn and epsilon 
    Inputs: screen - the screen to draw on 
            tmp - a tuple of X_array and epsilon array
'''
def drawRec(screen, boxes, obs, color):
    # color = (randint(1,255), randint(1,255), randint(1,255))
    for b in boxes:
        X0 = b.center
        epsilon0 = b.epsilon
        # pygame.draw.rect(screen, (max(255-i*10,0), min(i*10, 255), max(255-i*10,0)), (X0.state[0]-epsilon0, X0.state[1]-epsilon0, 2*epsilon0, 2*epsilon0))
        # pygame.draw.rect(screen, red, (X1.state[0]-epsilon1, X1.state[1]-epsilon1, 2*epsilon1, 2*epsilon1))
        s = pygame.Surface((2*epsilon0, 2*epsilon0))
        s.set_alpha(70)
        s.fill(color)
        screen.blit(s, (X0.state[0]-epsilon0, X0.state[1]-epsilon0))
        # print 'Box centered at ', (X0.state[0], X0.state[1]), 'with radius ', epsilon0
        # boxChecker(X0, epsilon0, obs)
        pygame.display.update()

    # wait = raw_input('---------- PAUSED, press ENTER to continue ----------')
    drawEnd()

'''
    This function draws a given node specified by t(type) on the screen
    This function is for debug purposes. 
    Inputs: screen - the screen to draw on
            Xn - the node to be drawn
            t - type; 1 - Xrand; 2 - Xnear
'''
def drawNode(screen, Xn, t):
    x = int(Xn[0])
    y = int(Xn[1])
    if t == 1:
        pygame.draw.circle(screen, green, (x,y), 1)
    elif t == 2:
        pygame.draw.circle(screen, red, (x,y), 1)

def drawEnd():
    for e in pygame.event.get():
	   if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
	        sys.exit("Leaving because you requested it.")
