import numpy as np
import cv2
import heapq

########## Defining the map width and height #############

width = 1200
height = 500

########## Defining Functions to Generate Action Set

# Function to perform upward action
def MoveU(x_pos,y_pos,cost):
    if y_pos == height-1:
        return None
    else:
        cost = 1
        return x_pos, y_pos+1, cost


# Function to perform downward action
def MoveD(x_pos,y_pos,cost):
    if y_pos == 0:
        return None
    else:
        cost = 1
        return x_pos, y_pos-1, cost


# Function to perform left action
def MoveL(x_pos,y_pos,cost):
    if x_pos == 0:
        return None

    else:
        cost = 1
        return x_pos - 1, y_pos, cost
    

# Function to perform right action
def MoveR(x_pos,y_pos,cost):
    if x_pos == width-1:
        return None
    else:
        cost = 1
        return x_pos + 1, y_pos, cost


# Function to perform up-right action
def MoveUR(x_pos,y_pos,cost):
    if y_pos == height-1 or x_pos == width-1:
        return None
    else:
        cost = 1.4
        return x_pos + 1, y_pos + 1, cost


# Function to perform down-right action
def MoveDR(x_pos,y_pos,cost):
    if y_pos == 0 or x_pos == width-1:
        return None

    else:
        cost = 1.4
        return x_pos + 1, y_pos - 1, cost
    

# Function to perform up-left action
def MoveUL(x_pos,y_pos,cost):
    if y_pos == height-1 or x_pos == 0:
        return None

    else:
        cost = 1.4
        return x_pos - 1, y_pos + 1, cost
    

# Function to perform down-left action    
def MoveDL(x_pos,y_pos,cost):
    if y_pos == 0 or x_pos == 0:
        return None

    else:
        cost = 1.4
        return x_pos - 1, y_pos - 1, cost
    
# Function to check if the current node is a goal node    
def goal_check(x_current,y_current, x_goal, y_goal):
    if (x_current, y_current) == (x_goal, y_goal):
        return True