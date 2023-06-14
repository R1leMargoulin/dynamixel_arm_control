from ikpy import chain
import numpy as np

ros_path = "/home/r1/ros2_ws/src/"

# Define the robot's dimensions and joint limits
link_lengths = [0.135, 0.149, 0.160, 0.065]
joint_limits = [(-2.9671, 2.9671), (-1.7016, 1.7016), (-2.147, 2.147), (-3.0541, 3.0541)]

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

    init_joints = [0,0,0,0,0,0]

    target1 = [0.1, 0.1, 0.2] 
    target2 = [0.1, 0.1, 0.15]

    frame_target1 = np.eye(4)
    frame_target1[:3,3] = target1

    frame_target2 = np.eye(4)
    frame_target2[:3,3] = target2

#     joint_angles = calculate_inverse_kinematics(x, y, z)
#     if joint_angles is not None:
#         print("Joint angles:", joint_angles)
    print (robot)
    # print(str(x)+"/"+str(y)+"/"+str(z))

    #kinematics calculations
    # solution = calculate_inverse_kinematics(x,y,z)
    solution = frame_inverse_kinematics(frame_target1, init_joints)
    #print(solution)
    j1 = solution[1]
    j2 = solution[2]
    j3 = solution[3]
    j4 = solution[4]
    print([j1,j2,j3,j4])

    solution = frame_inverse_kinematics(frame_target2, [0,j1,j2,j3,j4,0])
    #print(solution)
    j1 = solution[1]
    j2 = solution[2]
    j3 = solution[3]
    j4 = solution[4]
    print([j1,j2,j3,j4])