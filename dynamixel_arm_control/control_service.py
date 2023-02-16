from dynamixel_arm_srv.srv import MoveitController
from dynamixel_arm_srv.srv import MoveJoints
from dynamixel_arm_control import joints_controller

import rclpy
from rclpy.node import Node


class movePlanningService(Node):

    def __init__(self):
        super().__init__('move_planning_service')
        self.srv_planning = self.create_service(MoveitController, '/move_planning_service', self.move_planning_callback)
        self.srv_joints = self.create_service(MoveJoints, 'move_joints', self.move_joints_callback)
        self.robot = joints_controller.DynamixelArm()
        self.robot.start()

    def move_planning_callback(self, request, response):
        print("order recieved")
        self.get_logger().info('Moving_Robot...' )
        #print(request.motor_ids)
        #print(request.goal_poses) 
        #print(request.speeds) 
        #print(request.accelerations)
        try:
            self.move_planning_position(request.motor_ids, request.goal_poses, request.speeds, request.accelerations)
            print(" j'arrive ici")
            response.response = "OK"
            return response
        except Exception as e: 
            print("error")
            print(e)
            response.response = "ERROR"
            self.robot.stop()
            return response
        
    def move_joints_callback(self, request, response):
        self.get_logger().info('Moving_Robot...' )
        try:
            self.robot.move_joints(request.joint1, request.joint2 ,request.joint3 ,request.joint4)
            response.response = "OK"
            return response
        except:
            self.robot.stop()

#-------------------------------------CONVERTS---------------------------------------------
    #ACCELERATION
    def acc_revMin2_to_radSec2(self, acceleration):
        return acceleration * 0.00175

    def acc_radSec2_to_revMin2(self, acceleration):
        return acceleration / 0.00175

    #Speed
    def speed_radSec_to_revMin(self, acceleration):
        return acceleration / 0.10472
    
    def speed_revMin_to_radSec(self, acceleration):
        return acceleration * 0.10472
    
#------------------------------------MOVEMENTS_PLANNING---------------------------------------------

    def move_planning_position(self, motorIds, GoalPoses, Speeds, Accelerations):
        for j in range(len(motorIds)):
            #print("speed : " + str(self.speed_radSec_to_revMin(Speeds[j])))
            #print("acceleration :" + str(self.acc_radSec2_to_revMin2(Accelerations[j])))
            self.robot.writeProfileVelocity(motorIds[j], self.speed_radSec_to_revMin(Speeds[j]))
            #self.robot.writeProfileAcceleration(motorIds[j], self.acc_radSec2_to_revMin2(Accelerations[j]))
        for j in range(len(motorIds)):
            #print(self.robot.robot_infos["j"+str(motorIds[j])])
            self.robot.writeGoalPos(motorIds[j],self.robot.rad2Dynamixel(self.robot.robot_infos["j"+str(motorIds[j])], GoalPoses[j]))

def main(args=None):

    rclpy.init(args=args)
    srv = movePlanningService()

    try:
        rclpy.spin(srv)
        srv.destroy_node()
        rclpy.shutdown()
    except:
        srv.robot.stop()


if __name__ == '__main__':
    main()