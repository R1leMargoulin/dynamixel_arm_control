import rclpy
from rclpy.qos import QoSProfile
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_commander import MoveGroupCommander

def main():
    rclpy.init()

    # Create a node
    node = rclpy.create_node('trajectory_publisher')

    # Create a MoveGroupCommander for the arm
    move_group = MoveGroupCommander("manipulator")

    # Set the planning time
    move_group.set_planning_time(5.0)  # Adjust the planning time as needed

    # Get the joint names
    joint_names = move_group.get_joint_names()

    # Create a JointTrajectory message to specify the trajectory
    trajectory = JointTrajectory()
    trajectory.joint_names = joint_names

    # Set the time from start for the trajectory points (in seconds)
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[0].time_from_start.sec = 0
    trajectory.points[0].time_from_start.nanosec = 0

    joint_goals = [1.0,0.0,0.0,0.0]
    
    # Assuming you have a list of joint goals stored in the 'joint_goals' variable
    for joint_goal in joint_goals:
        point = JointTrajectoryPoint()
        point.positions = joint_goal
        trajectory.points.append(point)

    # Execute the trajectory
    move_group.execute(trajectory, wait=True)

    # Cleanup
    rclpy.shutdown()

if __name__ == '__main__':
    main()
