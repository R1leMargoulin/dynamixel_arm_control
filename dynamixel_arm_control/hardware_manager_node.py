#-----------------------------------------IMPORTS---------------------------------------------
import rclpy
#import threading
from rclpy.node import Node

from dynamixel_arm_msgs.msg import DynamixelPosition
from dynamixel_arm_msgs.msg import DynamixelOrder
from dynamixel_arm_control import DynamixelArm



class movePlanningService(Node):


#-------------------------------------------INIT---------------------------------------------
    def __init__(self):
        super().__init__('move_planning_service')
        
        #hardware
        self.robot = DynamixelArm.DynamixelArm()
        self.robot.start()
        #listener
        self.order_listener = self.create_subscription(DynamixelOrder, '/hardware_order', self.order_callback, 10)

        #publishers
        self.motorPosition_publisher = self.create_publisher(DynamixelPosition, '/dynamixel_position', 10)

     

#-------------------------------------SERVICES-----------------------------------------------

    def order_callback(self, request, response):
        print("order recieved")
        try:
            if(request.order_type == "write"):
                self.robot.write(request.id, request.data, request.nb_bytes, request.table_address)
            elif(request.order_type == "read_joint"):
                #read motor pose
                response = self.robot.read(request.id, request.nb_bytes, request.table_address)
                #publish motor pose
                msg = DynamixelPosition()
                msg.id = request.id
                msg.pose = response
                self.motorPosition_publisher.publish(msg)
            else :
                self.get_logger().info('ERROR IN ORDER TYPE')
        except Exception as e: 
            print("error")
            print(e)
            self.robot.stop()
            self.get_logger().info('I heard: "%s"' % str(request))


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