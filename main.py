#### import robot files
import robkin
import robtest
import robtime
import pickle

#### import python stuff
import warnings

#####################################
### variables stuff
#####################################
tolerance = .01
checkStartPosition = [-2,-2,-2] # must be ints. x has to equal y. must be the very corner of the bottom left quadrant.
conversion = 100 # 100 -> m to cm, 1000 -> m to mm.....

### start timer
startTime = robtime.get_time()

#####################################
# initiates/loads robot
# input: filename of the robot
# output: robot_chain
######################################

#-------------------------------------------------------------------------------------------------------------------
#### results for robotarmv2
#-------------------------------------------------------------------------------------------------------------------
with warnings.catch_warnings():
    warnings.filterwarnings("ignore", category=UserWarning)
    robotarmv2_chain = robkin.loadRobot("robotarmv2meters.urdf")
    print("Caution! UserWarnings Suppressed")

robotReachBool, robotReachNum = robtest.checkReach3D(robotarmv2_chain, tolerance, checkStartPosition, conversion,True, True, False)
robtest.writeToFile3D(robotReachBool, "robotReachBoolv2_3D.txt")
robtest.writeToFile3D(robotReachNum, "robotReachNumv2_3D.txt")













