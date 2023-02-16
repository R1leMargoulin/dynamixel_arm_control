from dynamixel_arm_srv.srv import MoveJoints
from dynamixel_arm_control import joints_controller

import rclpy
from rclpy.node import Node


class moveJointsService(Node):

    def __init__(self):
        super().__init__('move_joints_service')
        self.srv = self.create_service(MoveJoints, 'move_joints', self.move_joints_callback)
        self.robot = joints_controller.DynamixelArm()
        self.robot.start()

    def move_joints_callback(self, request, response):
        self.get_logger().info('Moving_Robot...' )
        try:
            self.robot.move_joints(request.joint1, request.joint2 ,request.joint3 ,request.joint4)
            return response
        except:
            self.robot.stop()


def main(args=None):
    rclpy.init(args=args)

    srv = moveJointsService()

    rclpy.spin(srv)

    rclpy.shutdown()


if __name__ == '__main__':
    main()