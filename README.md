# RRT Planner
Implementation of RRT path planner and several dynamic models

## Bicycle model
A bicycle dynamic car model. 

Source Paper: http://www.cs.cmu.edu/~motionplanning/reading/PlanningforDynamicVeh-1.pdf

Note: there are some mistakes in the paper. 

1) r is always yaw rate; not tire radius
2) the matrix in equation(4) should be [A B C D]

## Quadrotor model
A Quadcopter dynamic model. 

Source Paper: https://easychair.org/publications/open/sr6 model 3.3
