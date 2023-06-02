import rospy
import time

from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import RobotState, Constraints, JointConstraint, MotionPlanRequest

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from dynamixel_arm_srv.srv import TrajectoryMoveit


class TrajectoryGoalsClient():

    def __init__(self):
        rospy.init_node('trajectory_goals_client', anonymous=True)
        rospy.wait_for_service('/plan_kinematic_path')
        self.motion_plan_client = rospy.ServiceProxy('/plan_kinematic_path', GetMotionPlan)
        
        rospy.wait_for_service('/joint_trajectory_follow')
        self.bridgeClient = rospy.ServiceProxy('/joint_trajectory_follow', TrajectoryMoveit)


    # def send_goal(self, j1,j2,j3,j4): #j1/j2/j3/j4 floats in rad.
    #     request = GetMotionPlan.Request()

    #     # Set the joint names for the goal
    #     request.group_name = 'manipulator'
    #     request.num_planning_attempts = 1
    #     request.allowed_planning_time = 1.0
    #     request.max_velocity_scaling_factor = 0.10
    #     request.max_acceleration_scaling_factor = 0.10

    #     #creating start state
    #     # Set the start state
    #     start_state = RobotState()
    #     start_state.joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4']
    #     start_state.joint_state.position = [0.0, 0.0, 0.0, 0.0]  # Set the joint positions for the start state
    #     request.start_state = start_state


    #     # Prepare your trajectory goals
    #     trajectory = JointTrajectory()
    #     trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

    #     # Create a goal point
    #     point = JointTrajectoryPoint()
    #     point.time_from_start.secs = 1  # Specify the time from start in seconds
    #     point.positions = [j1, j2, j3, j4]  # Set the joint positions for the goal
    #     trajectory.points.append(point)

    #     goal_constraints = Constraints()

    #     for i in range(len(trajectory.joint_names)):
    #         # Set joint constraints for the goal state
    #         joint_constraint = JointConstraint()
    #         joint_constraint.joint_name = trajectory.joint_names[i]
    #         joint_constraint.position = trajectory.points[0].positions[i]  # Specify the desired position constraint for joint1
    #         joint_constraint.tolerance_above = 0.01  # Specify the tolerance above the position constraint
    #         joint_constraint.tolerance_below = 0.01  # Specify the tolerance below the position constraint

    #         # Add the joint constraint to the goal constraints
    #         goal_constraints.joint_constraints.append(joint_constraint)

    #     # Set the goal constraints in the motion plan request
    #     request.goal_constraints.append(goal_constraints)

    #     # Call the service
    #     future = self.motion_plan_client.call_async(request)
    #     rclpy.spin_until_future_complete(self, future)

    #     if future.result() is not None:
    #         response = future.result()
    #         self.get_logger().info('Motion plan received: %s' % response.motion_plan_response.trajectory)
    #         # Process the response as needed
    #     else:
    #         self.get_logger().info('Failed to receive motion plan response.')


    def send_trajectory(self, trajectoryTable, start = [0.0, 0.0, 0.0, 0.0]): #j1/j2/j3/j4 floats in rad.
        start_pose = start
        go = True
        final_full_trajectory = JointTrajectory()

        for positions in trajectoryTable:
            request = MotionPlanRequest()

            # Set the joint names for the goal
            request.group_name = 'manipulator'
            request.num_planning_attempts = 1
            request.allowed_planning_time = 2.0
            request.max_velocity_scaling_factor = 0.50
            request.max_acceleration_scaling_factor = 0.50

            #creating start state
            # Set the start state
            start_state = RobotState()
            start_state.joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4']
            start_state.joint_state.position = start_pose  # Set the joint positions for the start state
            request.start_state = start_state


            # Prepare your trajectory goals
            trajectory = JointTrajectory()
            trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
            #print(trajectoryTable)
            # Create a goal point
        
            #print(positions)
            point = JointTrajectoryPoint()
            point.time_from_start.secs = 1
            point.positions = positions
            trajectory.points.append(point)

            goal_constraints = Constraints()

            for i in range(len(trajectory.joint_names)):
                # Set joint constraints for the goal state
                joint_constraint = JointConstraint()
                joint_constraint.joint_name = trajectory.joint_names[i]
                joint_constraint.position = positions[i]  # Specify the desired position constraint for joint1
                joint_constraint.tolerance_above = 0.01  # Specify the tolerance above the position constraint
                joint_constraint.tolerance_below = 0.01  # Specify the tolerance below the position constraint

                #print(joint_constraint.position)

                # Add the joint constraint to the goal constraints
                goal_constraints.joint_constraints.append(joint_constraint)

            # Set the goal constraints in the motion plan request
            request.goal_constraints.append(goal_constraints)

            #print(request.goal_constraints)

            # Call the service
            future = self.motion_plan_client(request)
            #rclpy.spin_until_future_complete(self, future)

            if future is not None: #ros1??
                #appeler l'action server ici
                response = future
                print("-------------------------------------")
                #self.get_logger().info('Motion plan received: %s' % response.motion_plan_response)
                # self.get_logger().info('time test BEFORE: %s' % response.motion_plan_response.trajectory.joint_trajectory.points[-1].time_from_start.secs)
                # self.get_logger().info('time test BEFORE: %s' % response.motion_plan_response.trajectory.joint_trajectory.points[-1].time_from_start.nsecs)
                #concatenate all of our results in one single trajectory
                current_trajectory = response.motion_plan_response.trajectory.joint_trajectory
                if (len(final_full_trajectory.points)==0):
                    #no add needed as it is the first point, we just append
                    final_full_trajectory.joint_names = current_trajectory.joint_names
                    for i in range(len(current_trajectory.points)):
                        if (i != len(current_trajectory.points) -1):
                            final_full_trajectory.points.append(current_trajectory.points[i])
                else:
                    #we add the previous time_from_start
                    previous_time_sec = final_full_trajectory.points[-1].time_from_start.secs
                    previous_time_nanosec = final_full_trajectory.points[-1].time_from_start.nsecs
                    for i in range(len(current_trajectory.points)):
                        current_trajectory.points[i].time_from_start.secs = previous_time_sec + current_trajectory.points[i].time_from_start.secs
                        if((current_trajectory.points[i].time_from_start.nsecs + previous_time_nanosec) >= 1000000000):
                            current_trajectory.points[i].time_from_start.nsecs = current_trajectory.points[i].time_from_start.nsecs + previous_time_nanosec - 1000000000
                            current_trajectory.points[i].time_from_start.secs = current_trajectory.points[i].time_from_start.secs + 1
                        else:
                            current_trajectory.points[i].time_from_start.nsecs = current_trajectory.points[i].time_from_start.nsecs + previous_time_nanosec
                        if ((i == len(current_trajectory.points) -1) or (i == 0)):
                            current_trajectory.points[i].velocities = final_full_trajectory.points[-1].velocities
                        final_full_trajectory.points.append(current_trajectory.points[i])
                start_pose = positions
                print('point sent: %s' % current_trajectory.points[i])
            else:
                print('Failed to receive motion plan response.')
                go = False
            
            # self.get_logger().info('time test AFTER: %s' % final_full_trajectory.points[-1].time_from_start.secs)
            # self.get_logger().info('time test AFTER: %s' % final_full_trajectory.points[-1].time_from_start.nsecs)
                    
                        
                # point_trajectory = TrajectoryMoveit.Request()
                # point_trajectory.trajectory = response.motion_plan_response.trajectory.joint_trajectory

        if(go==True):
            future_traj = self.bridgeClient(final_full_trajectory)
            #rclpy.spin_until_future_complete(self, future_traj)
            # print("debug mode")
                
            

    # def send_one_point_to_action

def main(args=None):
    trajectory_goals_client = TrajectoryGoalsClient()
    #rospy.spin()

    ##for one joint goal
    # trajectory_goals_client.send_goal(0.1,-1.0,0.0,0.0)
    # trajectory_goals_client.send_goal(1,0.0,0.0,0.0)

    #for a trajectory of several goal joints.


    

    
    # #small lists of poses for tests
    # goal_positions = [
    #         [0.0, 0.0, 0.0, 0.0],
    #         [1.0, 0.0, 0.0, 0.0],  # Goal 1
    #         [2.0, 0.0, 0.0, 0.0],  # Goal 2
    #         [1.0, 0.0, 0.0, 0.0],
    #         [0.0, 0.0, 0.0, 0.0]   # Goal 3
    #     ]
    


    #FULL LEMNISCATE TRAJECTORY TEST
    ros_path = "/home/r1/catkin_ws/src/"
    csv_directory = "/home/r1/"

    f = open(csv_directory+"joints_waypoints.csv", "r")
    csv = f.read()

    goal_positions = []
    joints_waypoints = []

    csv = csv.split("\n")
    csv.pop()
    for l in csv:
        point = l.split(";")
        #print (point)
        goal_positions.append([float(point[0]),float(point[1]),float(point[2]),float(point[3])])
    

    #go to an init pose
    #trajectory_goals_client.send_trajectory([goal_positions[0]])
    #time.sleep(0.5)

    for p in goal_positions:
        print(p)


    #SENDING THE ACTUAL LIST OF GOALS
    trajectory_goals_client.send_trajectory(goal_positions, start= goal_positions[0])

if __name__ == '__main__':
    main()