import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetMotionPlan
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import RobotState, Constraints, JointConstraint

class TrajectoryGoalsClient(Node):

    def __init__(self):
        super().__init__('trajectory_goals_client')
        self.motion_plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        while not self.motion_plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def send_goal(self, j1,j2,j3,j4): #j1/j2/j3/j4 floats in rad.
        request = GetMotionPlan.Request()

        # Set the joint names for the goal
        request.motion_plan_request.group_name = 'manipulator'
        request.motion_plan_request.num_planning_attempts = 1
        request.motion_plan_request.allowed_planning_time = 1.0
        request.motion_plan_request.max_velocity_scaling_factor = 0.10
        request.motion_plan_request.max_acceleration_scaling_factor = 0.10

        #creating start state
        # Set the start state
        start_state = RobotState()
        start_state.joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4']
        start_state.joint_state.position = [0.0, 0.0, 0.0, 0.0]  # Set the joint positions for the start state
        request.motion_plan_request.start_state = start_state


        # Prepare your trajectory goals
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        # Create a goal point
        point = JointTrajectoryPoint()
        point.time_from_start.sec = 1  # Specify the time from start in seconds
        point.positions = [j1, j2, j3, j4]  # Set the joint positions for the goal
        trajectory.points.append(point)

        goal_constraints = Constraints()

        for i in range(len(trajectory.joint_names)):
            # Set joint constraints for the goal state
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = trajectory.joint_names[i]
            joint_constraint.position = trajectory.points[0].positions[i]  # Specify the desired position constraint for joint1
            joint_constraint.tolerance_above = 0.01  # Specify the tolerance above the position constraint
            joint_constraint.tolerance_below = 0.01  # Specify the tolerance below the position constraint

            # Add the joint constraint to the goal constraints
            goal_constraints.joint_constraints.append(joint_constraint)

        # Set the goal constraints in the motion plan request
        request.motion_plan_request.goal_constraints.append(goal_constraints)

        # Call the service
        future = self.motion_plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info('Motion plan received: %s' % response.motion_plan_response.trajectory)
            # Process the response as needed
        else:
            self.get_logger().info('Failed to receive motion plan response.')


    def send_trajectory(self, trajectoryTable): #j1/j2/j3/j4 floats in rad.
        request = GetMotionPlan.Request()

        # Set the joint names for the goal
        request.motion_plan_request.group_name = 'manipulator'
        request.motion_plan_request.num_planning_attempts = 1
        request.motion_plan_request.allowed_planning_time = 5.0
        request.motion_plan_request.max_velocity_scaling_factor = 0.10
        request.motion_plan_request.max_acceleration_scaling_factor = 0.10

        #creating start state
        # Set the start state
        start_state = RobotState()
        start_state.joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4']
        start_state.joint_state.position = [0.0, 0.0, 0.0, 0.0]  # Set the joint positions for the start state
        request.motion_plan_request.start_state = start_state


        # Prepare your trajectory goals
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        # Create a goal point
        for positions in trajectoryTable:
            point = JointTrajectoryPoint()
            point.time_from_start.sec = 1
            point.positions = positions
            trajectory.points.append(point)

            goal_constraints = Constraints()

            for i in range(len(trajectory.joint_names)):
                # Set joint constraints for the goal state
                joint_constraint = JointConstraint()
                joint_constraint.joint_name = trajectory.joint_names[i]
                joint_constraint.position = trajectory.points[0].positions[i]  # Specify the desired position constraint for joint1
                joint_constraint.tolerance_above = 0.01  # Specify the tolerance above the position constraint
                joint_constraint.tolerance_below = 0.01  # Specify the tolerance below the position constraint

                # Add the joint constraint to the goal constraints
                goal_constraints.joint_constraints.append(joint_constraint)

            # Set the goal constraints in the motion plan request
            request.motion_plan_request.goal_constraints.append(goal_constraints)

            # Call the service
            future = self.motion_plan_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            if future.result() is not None:
                #appeler l'action server ici
                response = future.result()
                self.get_logger().info('Motion plan received: %s' % response.motion_plan_response)
                # Process the response as needed
            else:
                self.get_logger().info('Failed to receive motion plan response.')

    # def send_one_point_to_action

def main(args=None):
    rclpy.init(args=args)
    trajectory_goals_client = TrajectoryGoalsClient()

    ##for one joint goal
    # trajectory_goals_client.send_goal(0.1,-1.0,0.0,0.0)
    # trajectory_goals_client.send_goal(1,0.0,0.0,0.0)

    #for a trajectory of several goal joints.
    goal_positions = [
            [0.0, 0.0, 0.0, 0.0],  # Goal 1
            [1.0, -1.0, 0.0, 0.0],  # Goal 2
            [2.0, 0.0, 0.0, 0.0]   # Goal 3
        ]
    trajectory_goals_client.send_trajectory(goal_positions)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
