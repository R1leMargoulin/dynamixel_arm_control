import rospy

from dynamixel_arm_srv.srv import MoveitController
from dynamixel_arm_srv.srv import MoveJoints

from dynamixel_arm_msgs.msg import DynamixelOrder

from dynamixel_arm_control.hardware_infos import *




class GlobalController():
#-------------------------------------------INIT---------------------------------------------
    def __init__(self):
        #publishers
        self.order_publisher = rospy.Publisher('hardware_order', DynamixelOrder,  queue_size=10)
        rospy.init_node('control_node', anonymous=True)
        #services
        self.srv_planning = rospy.Service( 'move_planning_service', MoveitController, self.move_planning_callback)
        self.srv_joints = rospy.Service( 'move_joints', MoveJoints, self.move_joints_callback)

    def run(self):
        rospy.spin()



#-------------------------------------SERVICES-----------------------------------------------

    def move_planning_callback(self, request):
        print("order recieved")
        #self.get_logger().info('Moving_Robot...' )
        #print(request.motor_ids)
        #print(request.goal_poses) 
        #print(request.speeds) 
        #print(request.accelerations)
        try:
            self.move_planning_position(request.motor_ids, request.goal_poses, request.speeds, request.accelerations)
            print(" j'arrive ici")
            return "OK"
        except Exception as e: 
            print("error")
            print(e)
            return "ERROR"
        
    def move_joints_callback(self, request):
        #self.get_logger().info('Moving_Robot...' )
        print(request)
        jtable = [request.joint1, request.joint2, request.joint3, request.joint4]
        try:
            for i in range(len(jtable)):
                print("goal")
                print(i+1)
                msg = DynamixelOrder()
                msg.order_type = "write"
                msg.id = i+1
                msg.nb_bytes = NBBYTE_GOAL_POSITION
                msg.table_address = ADDR_GOAL_POSITION
                msg.data = self.rad2Dynamixel(robot_infos["j"+str(i+1)], jtable[i])
                self.order_publisher.publish(msg)
            return "OK"
        except Exception as e:
             print(e)
             response = "ERROR"
             return response

#-------------------------------------CONVERTS-----------------------------------------------
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
    

    #ANGLES
    def degree2Dynamixel(self, joint, angle):
        goal = angle*ANGLE_DEGREE_COEF+ joint["origin"]
        if goal < joint["pos_min"]:
            print("error value out of limits")
            return(joint["pos_min"])
        elif goal > joint["pos_max"]:
            print("error value out of limits")
            return(joint["pos_max"])
        else:
            return (goal)

    def dynamyxel2degree(self, joint, position):
        return ((position - joint["origin"])/ANGLE_DEGREE_COEF)

    def rad2Dynamixel(self, joint, angle):
        goal = angle*ANGLE_RAD_COEF+ joint["origin"]
        if goal < joint["pos_min"]:
            print("goal : "+str(goal))
            print(joint)
            print("error value min")
            return(joint["pos_min"])
        elif goal > joint["pos_max"]:
            print("error value max")
            print("goal : "+str(goal))
            print(joint)
            return(joint["pos_max"])
        else:
            return (int(goal))

    def dynamyxel2rad(self, joint, position):
        return ((position - joint["origin"])/ANGLE_RAD_COEF)
    

    #SPEEDS
    def speedForTable(self, speed):
        return int(abs(speed)/0.229) #convert the speed in rev/min to an int for table control -> 0.229(rev/min) per value
    
#----------------------------------MOVEMENTS_PLANNING----------------------------------------

    def move_planning_position(self, motorIds, GoalPoses, Speeds, Accelerations):
        for j in range(len(motorIds)):
            print(j)
            print("speed : " + str(self.speed_radSec_to_revMin(Speeds[j])))
            print("acceleration :" + str(self.acc_radSec2_to_revMin2(Accelerations[j])))
            msg = DynamixelOrder()
            msg.order_type = "write"
            msg.id = motorIds[j]
            msg.nb_bytes = NBBYTE_PROFILE_VELOCITY
            msg.table_address = ADDR_PROFILE_VELOCITY
            msg.data = self.speedForTable( self.speed_radSec_to_revMin(Speeds[j]))
            self.order_publisher.publish(msg)
        for j in range(len(motorIds)):
            #print(robot_infos["j"+str(motorIds[j])])
            msg = DynamixelOrder()
            msg.order_type = "write"
            msg.id = motorIds[j]
            msg.nb_bytes = NBBYTE_GOAL_POSITION
            msg.table_address = ADDR_GOAL_POSITION
            msg.data = self.rad2Dynamixel(robot_infos["j"+str(motorIds[j])], GoalPoses[j])
            self.order_publisher.publish(msg)





def main(args=None):

    node = GlobalController()
    node.run()

if __name__ == '__main__':
    main()
