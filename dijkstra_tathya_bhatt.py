import numpy as np
import cv2
import heapq
import time

start = time.time()

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

############### Taking User Inputs for Start and Goal Nodes #############

check = True

while check:
    x_start = int(input("Enter the initial X position (5 to 1194): "))
    y_start = int(input("Enter the initial Y Position (5 to 494): "))
    print("Your Start Node Is: ", x_start, y_start)
    # Converting the coordinates to the instructed coordinate system
    y_start = height-y_start-1
    # Checks if the given node is in the free space
    if map[y_start, x_start,0] == 0:
        cv2.circle(map,(x_start, y_start), 2, (0, 180, 0), -1)
        check = False
    # If the starting node is in obstacle space
    else:
        print("Starting Position is in the Obstacle space! Re-Enter the Position")

check = True

while check:
    x_goal = int(input("Enter the Destination X Position (5 to 1194):"))
    y_goal = int(input("Enter the Destination Y Position (5 to 494):"))

    print("Your Goal Node Is: ", x_goal, y_goal)
    # Converting the coordinates to the instructed coordinate system
    y_goal = height-y_goal-1 
    # Checks if the given node is in the free space
    if map[y_goal, x_goal,0] == 0:
        # Checks if the start node and goal node are same
        if (x_start, y_start) == (x_goal, y_goal):
            print("Error! Start and Goal Position Cannot be Same")
        else:
            check = False
    else:
        print("Starting Position is in the Obstacle space! Re-Enter the Position")

# Declaring possible moves
moves = [MoveU, MoveL, MoveR, MoveD, MoveDL, MoveDR, MoveUR, MoveUL]

# Initializing the visited dictionary because list of tuple is less efficient than dictionary
visited = {(x_start, y_start): True}
# Storing Child Nodes as Keys and Parent Nodes as its value
child2parent = {}
# Cost to come of a child node 
cost2come = {(x_start, y_start): 0}
reach_flag = False
# Initializing the open list as a heapqueue
q = []
i = 0

# Add a node with its cost to the heap queue
heapq.heappush(q, (0,x_start, y_start))

while q:

    # Tuple Unpacking parameters
    cst, x_pos, y_pos = heapq.heappop(q)

    # If the goal is found, generate the path and change the flag to True and end the algorithm
    if x_pos == x_goal and y_pos == y_goal:
        print("Goal Reached! Path Generated")
        path_gen = generate_path(x_start,y_start,x_pos,y_pos,child2parent)
        reach_flag = True
        break
    
    # If not goal, then perform the required moves
    for move in moves:
            # print(move)
            node = move(x_pos, y_pos, cst)
            if node is not None:
                new_x, new_y, new_cst = node

                # If X Node and Y Node is in the bounding box and inside the free-space
                if 0 <= new_x < width and 0 <= new_y < height and map[new_y, new_x, 0] == 0:
                    
                    # If the new node is not in visited dictionary
                    if (new_x, new_y) not in visited:
                        # Calculate total cost to come by adding the parent cost to the cost of action
                        total_cst = cost2come[(x_pos,y_pos)] + new_cst
                        # Add the generated node to the Priority Queue
                        heapq.heappush(q, (total_cst, new_x, new_y))
                        #Store the Child and Parents
                        child2parent[(new_x, new_y)] = (x_pos, y_pos)
                        # Add the cost to come for that respective child
                        cost2come[(new_x, new_y)] = total_cst
                        # Add the explored node to the visited dictionary
                        visited[(new_x, new_y)] = True

                    # If the node is already visited, then update the cost to come
                    elif cost2come[(new_x, new_y)] > cost2come[(x_pos,y_pos)] + new_cst:
                            cost2come[(new_x, new_y)] = cost2come[(x_pos,y_pos)] + new_cst
                            child2parent[(new_x, new_y)] = (x_pos, y_pos)

# If the goal is not reachable
if reach_flag == False:
     print("Goal out of bounds")

end = time.time()

################# Visualization ######################
# Creating a video object
path_vid = cv2.VideoWriter('path_final.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 50, (width, height))

# Displaying the start and goal position on the map
cv2.circle(map, (x_start,y_start), 6, (92,11,227),-1)
cv2.circle(map, (x_goal,y_goal), 6, (0, 165, 255),-1)


num_frames = 250
i = 0

# This code writes the video in every 250 frames
for (x,y) in visited.keys():
    i+=1
    map[y,x] = (225, 105, 65)
    if i == num_frames:
        path_vid.write(map)
        cv2.imshow("Path Video", map)
        cv2.waitKey(1) 
        i = 0

########### Drawing an actual path #############
cv2.circle(map, (x_start,y_start), 6, (92,11,227),-1)
cv2.circle(map, (x_goal,y_goal), 6 , (0, 165, 255),-1)

# Iterating through the optimal path and generating a line based path representation
for i in range(len(path_gen) - 1):
    point = path_gen[i]
    next_point = path_gen[i+1]
    cv2.imshow("Path Video", map)
    cv2.waitKey(1) 
    cv2.line(map, point, next_point, (0, 165, 255), thickness=2)
    path_vid.write(map)

last_frame = map

# Letting the frame stay as it is for few seconds to see the video more clearly
for _ in range(200): 
    path_vid.write(last_frame)

path_vid.release()
cv2.waitKey(0)
cv2.destroyAllWindows()

# Printing the runtime of the algorithm
print("Video Generated, Runtime is: ", (end-start))