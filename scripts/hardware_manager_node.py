#-----------------------------------------IMPORTS---------------------------------------------
import rospy
#import threading
from dynamixel_arm_control.dynamixel_arm import DynamixelArm

from dynamixel_arm_msgs.msg import DynamixelPosition
from dynamixel_arm_msgs.msg import DynamixelOrder




class HardwareManager():


#-------------------------------------------INIT---------------------------------------------
    def __init__(self):
        rospy.init_node('hardware_manager', anonymous=True)

        #hardware
        self.robot = DynamixelArm()
        self.robot.start()
        #listener
        rospy.Subscriber('/hardware_order', DynamixelOrder, self.order_callback)

        #publishers
        self.motorPosition_publisher = rospy.Publisher('/dynamixel_position', DynamixelPosition, queue_size=10)

     

#-------------------------------------SERVICES-----------------------------------------------

    def order_callback(self, request):
        #print("order recieved")
        #print(request)
        try:
            if(request.order_type == "write"):
                self.robot.enable_torque(request.id)
                self.robot.write(request.id, request.data, request.nb_bytes, request.table_address)
            elif(request.order_type == "read_joint"):
                #read motor pose
                res = self.robot.read(request.id, request.nb_bytes, request.table_address)
                #publish motor pose
                msg = DynamixelPosition()
                msg.id = request.id
                msg.pose = res
                #print(msg)
                self.motorPosition_publisher.publish(msg)
            else :
                self.get_logger().info('ERROR IN ORDER TYPE')
        except Exception as e: 
            print('error')
            print(e)
            self.robot.stop()
            #self.get_logger().info('I heard: "%s"' % str(request))


def main(args=None):
    node = HardwareManager()

    try:
        rospy.spin()
        node.robot.stop()
    except Exception as e:
        print('error')
        print(e)
        node.robot.stop()


if __name__ == '__main__':
    main()