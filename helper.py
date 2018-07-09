import numpy as np

# Helper Functions
'''
    This function returns the distance between two given states
    It uses Euclidean distance
'''
def dist(s1, s2):
    x1 = s1[0]
    y1 = s1[1]

    x2 = s2[0]
    y2 = s2[1]
    return np.sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2))