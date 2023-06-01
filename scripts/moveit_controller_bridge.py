import rospy
import time
from moveit_msgs.msg import DisplayTrajectory
from dynamixel_arm_msgs.msg import DynamixelOrder
from dynamixel_arm_srv.srv import MoveitController
from dynamixel_arm_srv.srv import TrajectoryMoveit

from array import array

class MoveitBridge():
    def __init__(self):
        rospy.init_node('moveit_controller_bridge')
        rospy.wait_for_service('move_planning_service')
        self.serviceClient = rospy.ServiceProxy( 'move_planning_service', MoveitController)
        #rospy.Subscriber('/joint_trajectory_follow', DisplayTrajectory, self.display_callback)

        self.trajectory_sevice = rospy.Service( 'joint_trajectory_follow', TrajectoryMoveit, self.execute_callback)    

    def run(self):
        rospy.spin()



    def execute_callback(self, goal_handle):
        #print("aaaaaaaaaaaaaaaah")
        # print(display_msg.trajectory[0].joint_trajectory.points)
        motorIDs = array('B',[])
        #enabling torque first
        msg_torque = DynamixelOrder()
        msg_torque.order_type = "torque_enable"
        msg_torque.id = 0
        msg_torque.nb_bytes = 0
        msg_torque.table_address = 0
        msg_torque.data = 0
        self.order_publisher.publish(msg_torque)
        #Moving robot now
        for name in goal_handle.trajectory.joint_names:
            #print(name[-1])
            if(name[-1]=="1" or name[-1] == "2" or name[-1] == "3" or name[-1] == "4"):
                motorIDs.append(int(name[-1]))
        #print(display_msg.trajectory[0].joint_trajectory.points)
        totalTime = goal_handle.trajectory.points[-1].time_from_start.sec + goal_handle.trajectory.points[-1].time_from_start.nanosec * 0.000000001
        nbSteps = len(goal_handle.trajectory.points)
        print("nb steps:", nbSteps)

        print(goal_handle.trajectory.points[-1].positions)

        for i in range (len(goal_handle.trajectory.points)):
            step = goal_handle.trajectory.points[i]
            #print(step)
            # print("positions")
            # print(step.positions)
            # print("velocities")
            # print(step.velocities)
            # print("accelerations")
            # print(step.accelerations)
            # print("-------------------")      
             # Wait for the service to become available
                       
            self.move(motorIDs, array('f', step.positions), array('f', step.velocities), array('f', step.accelerations))
            #print(totalTime)
            if(i!= len(goal_handle.trajectory.points) -1):
                time.sleep(totalTime/nbSteps) #time to wait between two steps
        return "OK"

    def move(self, motorIds, GoalPoses, Speeds, Accelerations):
        # request = MoveitController()
        # request.motor_ids = motorIds
        # request.goal_poses = GoalPoses
        # request.speeds = Speeds
        # request.accelerations = Accelerations 
       
        print("-----------------------------")
        print(motorIds)
        print(GoalPoses)
        print(Speeds)
        print(Accelerations)
        print("-----------------------------")
        # Call the service
        future = self.serviceClient(motorIds, GoalPoses, Speeds, Accelerations)

        # # Wait for the service call to complete
        # rclpy.spin_until_future_complete(self, future)
        # if future.result() is not None:
        #     self.get_logger().info('Result of service call: %d' % future.result().success)
    
def main(args=None):
    try:
        node = MoveitBridge()
        node.run()
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
