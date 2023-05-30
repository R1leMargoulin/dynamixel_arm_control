import rclpy
import time
from moveit_msgs.msg import DisplayTrajectory
from dynamixel_arm_msgs.msg import DynamixelOrder
from dynamixel_arm_srv.srv import MoveitController
from dynamixel_arm_srv.srv import TrajectoryMoveit
from control_msgs.action import FollowJointTrajectory
from rclpy.node import Node
from rclpy.action import ActionServer

from array import array

class MoveJointPlanExecutionCallback(Node):
    def __init__(self):
        super().__init__('moveit_controller_bridge')
        self._client = self.create_client(MoveitController, '/move_planning_service')
#-----------------------------------------------
        # self._display_sub = self.create_subscription(
        #     DisplayTrajectory,
        #     '/display_planned_path',
        #     self.display_callback,
        # 10)
#------------------------------------------------
        self._action_server = self.create_service(TrajectoryMoveit,
            '/joint_trajectory_follow',
            self.execute_callback)
        print("bbb")

        self.order_publisher = self.create_publisher(DynamixelOrder, '/hardware_order', 10)

    def execute_callback(self, goal_handle):
        print("aaaaaaaaaaaaaaaah")
        # print(display_msg.trajectory[0].joint_trajectory.points)
        motorIDs = array('B',[])
        if not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service not available, quitting...')
            print("service not available")
            return   
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
        for i in range (len(goal_handle.trajectory.points)):
            step = goal_handle.trajectory.points[i]
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
        request.goal_poses = [GoalPoses[0],GoalPoses[1],GoalPoses[2],GoalPoses[3]]
        request.speeds = [Speeds[0],Speeds[1],Speeds[2],Speeds[3],]
        request.accelerations = [Accelerations[0],Accelerations[1],Accelerations[2],Accelerations[3]] 
       

        # Call the service
        future = self._client.call_async(request)

        # # Wait for the service call to complete
        # rclpy.spin_until_future_complete(self, future)
        # if future.result() is not None:
        #     self.get_logger().info('Result of service call: %d' % future.result().success)
    
def main(args=None):
    rclpy.init(args=args)

    node = MoveJointPlanExecutionCallback()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

    # node.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()
