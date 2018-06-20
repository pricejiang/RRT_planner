import pygame, sys
from pygame.locals import *

WINSIZE = [500, 500]
white = (255, 240, 200)
black = (20, 20, 40)
red = (255, 0, 0)
green = (0, 255, 0)
blue = (0, 0, 255)

'''
    This function nitializes pygame interface
'''
def initDraw():
    pygame.init()
    screen = pygame.display.set_mode(WINSIZE)
    pygame.display.set_caption('Bicycle Model       Test        05/2018')
    
    screen.fill(black)
    return screen

'''
    This function updates current status of planner
'''
def drawScreen(screen, p1, p2, goal, obs):
    pygame.draw.rect(screen, red, (goal[0],goal[2],goal[1]-goal[0],goal[3]-goal[2]))
    pygame.draw.line(screen,white,p1,p2)
    for ob in obs:
        pygame.draw.line(screen, blue, ob[0], ob[1])
    pygame.display.update()

    drawEnd()

def drawPath(screen, path, goal, obs):
    screen.fill(black)
    for i in range(len(path)):
        if i == len(path)-1:
            break
        p1 = (path[i][0], path[i][1])
        p2 = (path[i+1][0], path[i+1][1])
        drawScreen(screen, p1, p2, goal, obs)
    
    wait = raw_input('---------- Press ENTER to continue ----------')

def drawRec(screen, tmp):
    X_array, epsilon_array = tmp
    for i in range(len(X_array)):
        X0 = X_array[i]
        epsilon0 = epsilon_array[i]
        pygame.draw.rect(screen, green, (X0.state[0]-epsilon0, X0.state[1]-epsilon0, 2*epsilon0, 2*epsilon0))
        # pygame.draw.rect(screen, red, (X1.state[0]-epsilon1, X1.state[1]-epsilon1, 2*epsilon1, 2*epsilon1))
        pygame.display.update()
    wait = raw_input('---------- Press ENTER to continue ----------')
    drawEnd()

def drawEnd():
    for e in pygame.event.get():
	   if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
	        sys.exit("Leaving because you requested it.")
