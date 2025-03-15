import ikpy.chain
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import warnings

#####################################
# initiates/loads robot
# input: filename of the robot
# output: robot_chain
######################################
def loadRobot(filename):
    # ---------------------------------------------------------
    # Load the chain from the URDF
    # ---------------------------------------------------------
    robot_chain = ikpy.chain.Chain.from_urdf_file(filename)

    # ---------------------------------------------------------
    # active_links_mask
    #    We want the four revolute joints to be 'True'
    # ---------------------------------------------------------
    mask_length = len(robot_chain.links)
    active_links_mask = [False]*mask_length

    # Based on your chain, the revolute joints might be at indices:
    #  1 base_motor_joint  (revolute, Z)
    #  2 motor_1_joint    (revolute, X)
    #  4 motor_2_joint    (revolute, X)
    #  6 motor_3_joint    (revolute, X)
    # The others are fixed or base
    active_links_mask[1] = True  # base_motor_joint
    active_links_mask[2] = True  # motor_1_joint
    active_links_mask[4] = True  # motor_2_joint
    active_links_mask[6] = True  # motor_3_joint

    robot_chain.active_links_mask = active_links_mask

    print(f"Robot '{filename}' loaded successfully.")
    return robot_chain

# ---------------------------------------------------------
# Inverse Kinematics
# input: target_position [x,y,z] ,robot_chain
# output: ik_solution
# ---------------------------------------------------------
def inKin(robot_chain, target_position, debug):
    if debug:
        print(f"\n------------------------------------------------------------------------------------------------------"
              f"\nSolving IK for target: {target_position}")
    ik_solution = robot_chain.inverse_kinematics(target_position)
    angles_deg = [round(math.degrees(a), 2) for a in ik_solution]
    if debug:
        print("IK Joint angles (deg):", angles_deg)
    return ik_solution

# ---------------------------------------------------------
# Forward Kinematics
# input: robot_chain, ik solution
# output: computer_pos with forward kinematics
# ---------------------------------------------------------
def fwKin(robot_chain, ik_solution, debug):
    fk_matrix = robot_chain.forward_kinematics(ik_solution)
    computed_pos = fk_matrix[:3, 3]
    angles_deg = [round(math.degrees(a), 2) for a in ik_solution]
    if debug:
        print(f"\nRunning forwards kinematics on (deg) {angles_deg} \nComputed position: {computed_pos}")
    return computed_pos

# ---------------------------------------------------------
# Checks to see if the IK worked
# input: computer_pos with fk, target_pos, tolerance, print info (True/False)
# ouput: true/false (if it worked or not), difference
# ---------------------------------------------------------
def ikCheck(computed_pos, target_position, tolerance, debug_info):
    success = True
    difference = [0,0,0]

    for i in range(3):
        difference[i] = abs(computed_pos[i] - target_position[i])
        if abs(computed_pos[i] - target_position[i]) > tolerance:
            success = False
    if success and debug_info:
        print(f"\nIK successful for {target_position}")
    if (success == False) and debug_info:
        print(f"\nIK Failed. \nDifference is {difference}")

    return success


# ---------------------------------------------------------
# Visualise IK results
# input: robot_chain, target_pos_ ik_solutions
# output: graphs...
# ---------------------------------------------------------
def visKinematics(robot_chain, target_position, ik_solution):

    print("Visualising Kinematic Results...")
    fig = plt.figure(figsize=(16, 12))

    # Subplot 1: 3D View
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    robot_chain.plot(ik_solution, ax=ax1, target=target_position)
    ax1.set_xlim(-2, 2)
    ax1.set_ylim(-2, 2)
    ax1.set_zlim(-2, 2)
    ax1.set_xlabel("X-axis")
    ax1.set_ylabel("Y-axis")
    ax1.set_zlabel("Z-axis")
    ax1.set_title("3D View")

    # Subplot 2: YZ plane
    ax2 = fig.add_subplot(2, 2, 2, projection='3d')
    robot_chain.plot(ik_solution, ax=ax2, target=target_position)
    ax2.view_init(elev=0, azim=90)  # Look along +X
    ax2.set_xlim(-2, 2)
    ax2.set_ylim(-2, 2)
    ax2.set_zlim(-2, 2)
    ax2.set_xlabel("Y-axis")
    ax2.set_ylabel("Z-axis")
    ax2.set_zlabel("X-axis")
    ax2.set_title("Projection on YZ Plane")

    # Subplot 3: XZ plane
    ax3 = fig.add_subplot(2, 2, 3, projection='3d')
    robot_chain.plot(ik_solution, ax=ax3, target=target_position)
    ax3.view_init(elev=0, azim=0)  # Look along +Y
    ax3.set_xlim(-2, 2)
    ax3.set_ylim(-2, 2)
    ax3.set_zlim(-2, 2)
    ax3.set_xlabel("X-axis")
    ax3.set_ylabel("Z-axis")
    ax3.set_zlabel("Y-axis")
    ax3.set_title("Projection on XZ Plane")

    # Subplot 4: Top View (XY plane)
    ax4 = fig.add_subplot(2, 2, 4, projection='3d')
    robot_chain.plot(ik_solution, ax=ax4, target=target_position)
    ax4.view_init(elev=90, azim=-90)  # Look down +Z
    ax4.set_xlim(-2, 2)
    ax4.set_ylim(-2, 2)
    ax4.set_zlim(-2, 2)
    ax4.set_xlabel("X-axis")
    ax4.set_ylabel("Y-axis")
    ax4.set_zlabel("Z-axis")
    ax4.set_title("Top View (XY Plane)")

    plt.tight_layout()
    plt.show()

def runAndCheckIK(robot_chain, target_position, tolerance, debug, graph):

    ik_solution = inKin(robot_chain, target_position, debug)
    fk_pos = fwKin(robot_chain, ik_solution, debug)
    ik_success = ikCheck(fk_pos, target_position, tolerance, debug)
    if graph:
        visKinematics(robot_chain, target_position, ik_solution)

    if ik_success:
        return ik_solution
    else:
        return None
