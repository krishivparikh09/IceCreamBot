# Robot Arm Kinematics and Reachability Testing

## Overview
This repository provides Python scripts for analyzing the kinematics and reachability of a 4-axis robotic arm using **Inverse Kinematics (IK)** and **Forward Kinematics (FK)**. It includes modules for loading a robot URDF, computing and validating IK solutions, and visualizing the robot's reachable workspace.

## Features
- **Inverse Kinematics (IK)**: Computes joint angles needed to reach a given position.
- **Forward Kinematics (FK)**: Computes end-effector position from given joint angles.
- **Reachability Analysis**: Determines if the robot can reach specific 3D coordinates.
- **3D Visualization**: Plots the robot's reachability in 2D and 3D.
- **File Logging**: Saves reachability results to text files.
- **Performance Tracking**: Measures execution time.

## Project Structure
```
.
├── main.py        # Main script to execute reachability tests
├── robkin.py      # Robot kinematics (IK, FK, visualization)
├── robtest.py     # Reachability testing and data storage
├── robtime.py     # Timing utility functions
├── README.md      # Project documentation (this file)
└── robotarmv2meters.urdf # Robot description file
```

## Installation
### Prerequisites
Ensure you have Python 3 and install the necessary dependencies:
```sh
pip install numpy matplotlib ikpy
```

## Usage
### 1. Load the Robot URDF
The `robkin.py` module loads the robot description file (.urdf):
```python
from robkin import loadRobot
robot_chain = loadRobot("robotarmv2meters.urdf")
```

### 2. Run Inverse Kinematics
Compute joint angles for a target position:
```python
from robkin import inKin
ik_solution = inKin(robot_chain, [0.5, 0.2, 0.3], debug=True)
```

### 3. Validate IK with Forward Kinematics
Verify if FK returns the expected position:
```python
from robkin import fwKin
computed_position = fwKin(robot_chain, ik_solution, debug=True)
```

### 4. Check Robot Reachability
Run a reachability test for a given workspace:
```python
from robtest import checkReach3D
robot_reach_bool, robot_reach_num = checkReach3D(robot_chain, 0.01, [-2, -2, -2], 100, True, True, False)
```

### 5. Visualize Reachability
To generate 2D and 3D reachability plots:
```python
from robtest import visualizeRobotReach3D
visualizeRobotReach3D(robot_reach_bool, 100, 100, 100)
```

### 6. Save Results to File
Store the reachability results in a text file:
```python
from robtest import writeToFile3D
writeToFile3D(robot_reach_bool, "robotReachBool_3D.txt")
```

## Modules Breakdown
### **1. `robkin.py` (Robot Kinematics)**
```python
- loadRobot(filename): Loads URDF and sets active joints.
- inKin(robot_chain, target_position, debug): Computes IK solutions.
- fwKin(robot_chain, ik_solution, debug): Computes FK positions.
- ikCheck(computed_pos, target_position, tolerance, debug_info): Validates IK accuracy.
- visKinematics(robot_chain, target_position, ik_solution): Visualizes IK solutions in 3D.
- runAndCheckIK(...): Runs IK, checks FK validation, and optionally visualizes results.
```

### **2. `robtest.py` (Reachability Analysis)**
```python
- checkReach(robot_chain, tolerance, start_position, conversion, visualise, debug, graph): Checks 2D reachability.
- checkReach3D(robot_chain, tolerance, start_position, conversion, visualise, debug, graph): Checks 3D reachability.
- writeToFile(array, filename): Saves results to a file.
- visualizeRobotReach(robot_reach_bool, end): Generates 2D plots.
- visualizeRobotReach3D(robot_reach_bool_3d, endX, endY, endZ): Generates 3D plots.
```

### **3. `robtime.py` (Timing Utilities)**
```python
- get_time(): Returns current timestamp.
- time_elapsed(start_time): Computes elapsed time.
```

## Example Execution
To run the full reachability analysis:
```sh
python main.py
```
This will:
1. Load the robot model.
2. Compute IK solutions.
3. Validate with FK.
4. Perform reachability tests.
5. Save and visualize results.

## Contributing
If you'd like to contribute, fork the repository and submit a pull request with improvements or bug fixes.

## License
Not licensed. Just credit me if you are using it in any way.

---
### Author: Krishiv
### Last Updated: March 2025
