from ikpy import chain
import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

ros_path = "/home/r1/ros2_ws/src/"
csv_directory = "/home/r1/"

f = open(csv_directory+"test.csv", "r")
csv = f.read()

xyz_points = []
joints_waypoints = []

csv = csv.split("\n")
csv.pop()
for l in csv:
    point = l.split(";")
    print (point)
    xyz_points.append([float(point[0]),float(point[1]),float(point[2])])

# Create the Open Manipulator X robot model
robot = chain.Chain.from_urdf_file(ros_path+"dynamixel_arm_description/urdf/arm.urdf", base_elements=["link1"], active_links_mask=[False,True,True,True,True,False])

# Function to calculate inverse kinematics
def calculate_inverse_kinematics(x, y, z):
    target_position = np.array([x, y, z])
    try:
        joint_angles = robot.inverse_kinematics(target_position)
        return joint_angles
    except Exception:
        print("Target position is not reachable.")

def frame_inverse_kinematics(target, init_joints):
    try:
        joint_angles = robot.inverse_kinematics_frame(target, init_joints)
        return joint_angles
    except Exception:
        print("Target position is not reachable.")

# Main program
if __name__ == "__main__":


#     joint_angles = calculate_inverse_kinematics(x, y, z)
#     if joint_angles is not None:
#         print("Joint angles:", joint_angles)
    print (robot)
    actual_joints = [0,0,0,0,0,0]
    for p in xyz_points:
        x=p[0]
        y=p[1]
        z=p[2]

        target = [x,y,z]
        frame_target = np.eye(4)
        frame_target[:3,3] = target

        solution = frame_inverse_kinematics(frame_target, actual_joints)
        #print(solution)
        j1 = solution[1]
        j2 = solution[2]
        j3 = solution[3]
        j4 = solution[4]
        print([j1,j2,j3,j4])
        joints_waypoints.append([j1,j2,j3,j4])
        actual_joints = [0, j1,j2,j3,j4, 0]
        

    csvFormat = ""
    for point in joints_waypoints:
        csvFormat = csvFormat + str(point[0])+";" + str(point[1])+";" + str(point[2])+";" + str(point[3])+"\n"
    print(csvFormat)

    f = open(csv_directory+"joints_waypoints.csv", "w")
    f.write(csvFormat)
    f.close()