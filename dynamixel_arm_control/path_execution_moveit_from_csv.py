#!/usr/bin/env python

import rclpy
from rclpy.node import Node
#import moveit_commander
from moveit_msgs.msg import MotionPlanRequest
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import JointConstraint
from sensor_msgs.msg import JointState

## END_SUB_TUTORIAL

class moveitPathOrder(Node):
#-------------------------------------------INIT---------------------------------------------
    def __init__(self):
        super().__init__('positions_nodes')
        #publishers
        self.publisher = self.create_publisher(MotionPlanRequest, '/motion_plan_request', 10)

        self.names = ['joint1','joint2','joint3','joint4']
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]

    def create_joint_state(self):
        js = JointState()
        js.name = ["joint1","joint2","joint3","joint4", "gripper", "gripper_sub"]
        js.position=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        js.header.frame_id = 'link1'
        return js

    def create_joint_constraint(self, name, pos,tol_a = 0.0001, tol_b = 0.0001, weight = 1.0):
        j = JointConstraint()
        j.joint_name = name
        j.position = pos
        j.tolerance_above = tol_a
        j.tolerance_below = tol_b
        j.weight = weight
        return j


#--------------------------------------PUBLISHERS--------------------------------------------
    def publish_joint_planning_order(self):
        
        for j in self.names:
            order = MotionPlanRequest()
            order.start_state.joint_state = self.create_joint_state()
            c1 = Constraints()
            c1.joint_constraints.append(self.create_joint_constraint("joint1",1.0))
            c1.joint_constraints.append(self.create_joint_constraint("joint2",0.0))
            c1.joint_constraints.append(self.create_joint_constraint("joint3",0.0))
            c1.joint_constraints.append(self.create_joint_constraint("joint4",0.0))
            order.goal_constraints.append(c1)
            order.pipeline_id = 'ompl'
            order.group_name = 'manipulator'
            order.num_planning_attempts = 10
            order.allowed_planning_time = 1.0
            order.max_velocity_scaling_factor=0.1
            order.max_acceleration_scaling_factor = 0.1
            order.max_cartesian_speed = 0.0

            self.publisher.publish(order)


def main(args=None):
    rclpy.init(args=args)

    print("aaah")
    try:
        robot = moveitPathOrder()
        robot.publish_joint_planning_order()
    except Exception as e:
      print("error")
      print(e)
      rclpy.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
  main()
