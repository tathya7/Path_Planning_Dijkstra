# Path Planning using Dijkstra Algorithm

This project implements Dijkstra Algorithm using Python and OpenCV to find the optimal path from a start position to a goal position on a obstacle map and generates a nice smooth animation of the explored nodes and the optimal path using backtracking.


## Prerequisites

To run the code, ensure you have the following libraries installed:

- `numpy`
- `cv2` (OpenCV)
- `heapq`
- `time`

## Usage

1. Once you've installed the required libraries, you can run the code. It will prompt you to choose a starting position and a goal position. Keep in mind the allowed ranges, which consider the clearance of the map dimensions.

2. Make sure to select positions within these ranges to ensure the path is generated correctly and to avoid obstacles.

3. In the provided video, the start position is (80, 50), and the goal position is (600, 400).

4. Once the simulation begins, it will continue until the goal is reached. At that point, a simulation video will be created as output.

5. You can find all the necessary files and implementations in the provided GitHub repository.

## Github Repository

Added the TAs as collaborators and the link for the github repo is : https://github.com/tathya7/Path_Planning_Dijkstra.git