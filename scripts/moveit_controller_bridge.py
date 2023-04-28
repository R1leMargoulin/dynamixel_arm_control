import rospy
import time
from moveit_msgs.msg import DisplayTrajectory
from dynamixel_arm_srv.srv import MoveitController

from array import array

class MoveitBridge():
    def __init__(self):
        rospy.init_node('moveit_controller_bridge')
        rospy.wait_for_service('/move_planning_service')
        self.serviceClient = rospy.ServiceProxy( '/move_planning_service', MoveitController)
        rospy.Subscriber(
            '/display_planned_path',
            DisplayTrajectory,
            self.display_callback)

    def display_callback(self, display_msg):
        # print(display_msg.trajectory[0].joint_trajectory.points)
        motorIDs = array('B',[])
        for name in display_msg.trajectory[0].joint_trajectory.joint_names:
            #print(name[-1])
            motorIDs.append(int(name[-1]))
        #print(display_msg.trajectory[0].joint_trajectory.points)
        totalTime = display_msg.trajectory[0].joint_trajectory.points[-1].time_from_start.sec + display_msg.trajectory[0].joint_trajectory.points[-1].time_from_start.nanosec * 0.000000001
        nbSteps = len(display_msg.trajectory[0].joint_trajectory.points)
        for i in range (len(display_msg.trajectory[0].joint_trajectory.points)):
            step = display_msg.trajectory[0].joint_trajectory.points[i]
            print(step)
            print("positions")
            print(step.positions)
            # print("velocities")
            # print(step.velocities)
            # print("accelerations")
            # print(step.accelerations)
            # print("-------------------")      
             # Wait for the service to become available
                       
            self.move(motorIDs, array('f', step.positions), array('f', step.velocities), array('f', step.accelerations))
            print(totalTime)
            time.sleep(totalTime/nbSteps) #time to wait between two steps

    def move(self, motorIds, GoalPoses, Speeds, Accelerations):
        request = MoveitController.Request()
        request.motor_ids = motorIds
        request.goal_poses = GoalPoses
        request.speeds = Speeds
        request.accelerations = Accelerations 
       

        # Call the service
        future = self.serviceClient(request)

        # # Wait for the service call to complete
        # rclpy.spin_until_future_complete(self, future)
        # if future.result() is not None:
        #     self.get_logger().info('Result of service call: %d' % future.result().success)
    
def main(args=None):

    node = MoveitBridge()
    rospy.spin()


if __name__ == '__main__':
    main()
