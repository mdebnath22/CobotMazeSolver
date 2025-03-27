# PATHPLANNING: Maze Solver with Camera and MyCobot 600 Pro Integration

## Overview
This project implements a maze-solving algorithm that uses a camera to capture maze images and a Cobot 600 Pro for robotic path execution. It employs image processing, pathfinding algorithms, and inverse kinematics to enable an end-to-end solution for navigating mazes and mapping paths to robotic arm movements.

---

## Features
- **Maze Image Processing:** Captures and processes maze images using OpenCV.
- **Pathfinding Algorithm:** Implements the A* algorithm with cost optimization.
- **Path Simplification:** Simplifies the solution path using the Douglas-Peucker algorithm.
- **Camera Calibration:** Maps maze waypoints to robot coordinates.
- **Inverse Kinematics:** Computes robot joint angles for path execution.
- **Visualization:** Displays the maze solution path and robot movement.

---

## Prerequisites

### Hardware
- **Cobot 600 Pro**
- Camera for capturing maze images

### Software
- Python 3.8+
- MATLAB (for inverse kinematics)
- Required libraries: OpenCV, NumPy, pandas

---

## Installation

1. **Clone the Repository**
   ```bash
   git clone https://github.com/yourusername/maze-solver-cobot.git
   ```

2. **Install Python Dependencies**

3. **Configure MATLAB Environment**
   - Ensure MATLAB is installed and set up with the Robotics Toolbox.
   - Add the path to the Cobot URDF file in the MATLAB script.

4. **Connect Hardware**
   - Ensure the camera and Cobot 600 Pro are properly connected to your system.

---

## How to Use

### Maze Solving
1. **Run the Maze Solver Script**
   ```bash
   python maze.py
   ```
2. **Capture Maze Image**
   - Use the camera feed to capture the maze image by pressing 's'.
3. **Select Points**
   - Click to select the start and end points on the maze image.
4. **View Solution**
   - The simplified solution path will be displayed and saved to a CSV file.

## Key Functions

### Python Scripts
- **`preprocess_maze(image)`**
  Converts the maze image to a binary format.

- **`find_path_a_star_with_cost()`**
  Implements the A* algorithm to find an optimized path.

- **`simplify_path()`**
  Reduces unnecessary waypoints in the solution path.

- **`visualize_solution()`**
  Displays the solution path on the maze image.

- **`map_to_robot_coordinates()`**
  Maps pixel coordinates to robot coordinates.

### MATLAB Scripts
- **`robotIK.m`**
  Computes inverse kinematics for each waypoint and visualizes the robot's movement.

---

## Example Workflow
1. Capture maze image and process it.
2. Compute the solution path using A* and simplify it.
3. Map the path to robot coordinates using camera calibration.
4. Execute the path using inverse kinematics on the Cobot 600 Pro.

---

## Output Files
- **`waypoints.csv`**: Simplified maze waypoints in pixel coordinates.
- **`converted_coordinates.txt`**: Robot coordinates for the waypoints.
- **`angles.csv`**: Joint angles for robot path execution.

---

For any questions, issues, or contributions, feel free to open an issue or submit a pull request!

