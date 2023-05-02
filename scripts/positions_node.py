import rospy

from sensor_msgs.msg import JointState

from dynamixel_arm_msgs.msg import DynamixelOrder
from dynamixel_arm_msgs.msg import DynamixelPosition
from dynamixel_arm_control.hardware_infos import *




class PositionsNode():


#-------------------------------------------INIT---------------------------------------------
    def __init__(self):
        rospy.init_node('positions_node', anonymous=True)
        #publishers
        self.rate = rospy.Rate(20) # 20 Hz for t/cycle = 0.05sec or 50ms
        self.order_publisher = rospy.Publisher('hardware_order', DynamixelOrder, queue_size=10)
        self.joints_position_publisher = rospy.Publisher('joint_states', JointState, queue_size=10)

        #subscribers
        self.position_return = rospy.Subscriber('dynamixel_position', DynamixelPosition, self.joint_position_return_callback)

        self.names = ['joint1','joint2','joint3','joint4']
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]
    
    def run(self):
        while not rospy.is_shutdown():
            self.joints_read_order()
            self.joint_states_publish()
            self.rate.sleep()

        rospy.spin()

#----------------------------------SUBSCRIBER CALLBACKS--------------------------------------------

    def joint_position_return_callback(self, msg):
        id = msg.id
        joint = robot_infos["j"+str(id)]
        pose =  self.dynamyxel2rad(joint, msg.pose)
        self.joint_positions[id-1] = pose


#--------------------------------------PUBLISHERS--------------------------------------------
    def joints_read_order(self):
        for j in self.names:
            order = DynamixelOrder()
            order.order_type = "read_joint"
            order.id = int(j[-1])
            order.nb_bytes = NBBYTE_PRESENT_POSITION
            order.table_address = ADDR_PRESENT_POSITION
            self.order_publisher.publish(order)


    def joint_states_publish(self):
        msg = JointState()
        msg.name = self.names
        msg.position = self.joint_positions
        #print (msg.position)
        self.joints_position_publisher.publish(msg)


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




def main(args=None):

    
    node = PositionsNode()
    node.run()



if __name__ == '__main__':
    main()