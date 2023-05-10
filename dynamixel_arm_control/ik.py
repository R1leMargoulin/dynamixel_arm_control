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

# Main program
if __name__ == "__main__":
#     x = float(input("Enter the x coordinate: "))
#     y = float(input("Enter the y coordinate: "))
#     z = float(input("Enter the z coordinate: "))
    x=0.1
    y=0.1
    z=0.2

#     joint_angles = calculate_inverse_kinematics(x, y, z)
#     if joint_angles is not None:
#         print("Joint angles:", joint_angles)
    print (robot)
    print(str(x)+"/"+str(y)+"/"+str(z))
    solution = calculate_inverse_kinematics(x,y,z)
    #print(solution)
    j1 = solution[1]
    j2 = solution[2]
    j3 = solution[3]
    j4 = solution[4]
    print([j1,j2,j3,j4])