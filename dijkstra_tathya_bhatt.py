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
    
# Function which generates path after the goal has been reached
def generate_path(x_start,y_start,x_target, y_target, dic):
    path = []
    path.append((x_target, y_target))
    x,y = x_target,y_target
    while (x,y) in dic:
        path.append((x,y))
        (x,y) = dic[(x,y)]

    path.append((x_start,y_start))
    path.reverse()

    return path

############## Map Generation with Obstacles ############
    
# Create a white canvas which represents the clearance
map = np.full((500, 1200, 3), 255, dtype=np.uint8)
# Create a black canvas
cv2.rectangle(map, (5, 5), (1195, 495), (0, 0, 0), thickness=cv2.FILLED)

center1 = (650,250)
length1 = 155
# Define the center and size of the hexagon
center2 = (650, 250)
length2 = 150

vertices1 = []
for i in range(6):
    x1 = int(center1[0] + length1 * np.cos(np.deg2rad(60 * i - 30)))
    y1 = int(center1[1] + length1 * np.sin(np.deg2rad(60 * i - 30)))
    vertices1.append((x1, y1))

# Calculate the vertices of the hexagon
vertices2 = []
for i in range(6):
    x2 = int(center2[0] + length2 * np.cos(np.deg2rad(60 * i - 30)))
    y2 = int(center2[1] + length2 * np.sin(np.deg2rad(60 * i - 30)))
    vertices2.append((x2, y2))

# Convert vertices to numpy array
pts1 = np.array(vertices1, np.int32)
pts1 = pts1.reshape((-1, 1, 2))

# Convert vertices to numpy array
pts2 = np.array(vertices2, np.int32)
pts2 = pts2.reshape((-1, 1, 2))

cv2.fillPoly(map, [pts1], color=(255, 255, 255))
cv2.fillPoly(map, [pts2], color=(0, 180, 0))

################## RECT1 ##################################
cv2.rectangle(map, (95, 0), (180, 405), (255, 255, 255), thickness=cv2.FILLED)
cv2.rectangle(map, (100, 0), (175, 400), (0, 180, 0), thickness=cv2.FILLED)

################## RECT2 ##################################
cv2.rectangle(map, (270, 500), (355, 95), (255, 255, 255), thickness=cv2.FILLED)
cv2.rectangle(map, (275, 500), (350, 100), (0, 180, 0), thickness=cv2.FILLED)



#################### C SHAPE ##############################

cv2.rectangle(map, (895, 455), (1105, 45), (255, 255, 255), -1)
cv2.rectangle(map, (900, 450), (1100, 50), (0, 180, 0), -1)

# cv2.rectangle(map, (900, 450), (1100, 50), (0, 180, 0), -1)
cv2.rectangle(map, (895, 125), (1020, 375), (255, 255, 255), -1)
cv2.rectangle(map, (895, 130), (1015, 370), (0, 0, 0), -1)