#!/usr/bin/env python
# -*- coding: utf-8 -*-


import os
from math import pi
from ast import literal_eval

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import * # Uses Dynamixel SDK library

#********* DYNAMIXEL Model definition *********


class DynamixelArm:
    def __init__(self):

        self.MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430



        # Control table address

        self.ADDR_TORQUE_ENABLE          = 64
        self.ADDR_GOAL_POSITION          = 116
        self.ADDR_PRESENT_POSITION       = 132
        self.ADDR_PROFILE_VELOCITY       = 112
        self.ADDR_PROFILE_ACCELERATION   = 108
        self.NBBYTE_GOAL_POSITION        = 4
        self.NBBYTE_PROFILE_VELOCITY     = 4
        self.NBBYTE_PROFILE_ACCELERATION = 4
        self.DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
        self.DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
        self.BAUDRATE                    = 57600

        self.ANGLE_DEGREE_COEF = 4096/360
        self.ANGLE_RAD_COEF = 4096/(2*pi)

        self.PROTOCOL_VERSION            = 2.0
        self.DEVICENAME                  = '/dev/ttyACM0'

        self.TORQUE_ENABLE               = 1     # Value for enabling the torque
        self.TORQUE_DISABLE              = 0     # Value for disabling the torque
        self.DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold

        self.index = 0
        self.dxl_goal_position = [self.DXL_MINIMUM_POSITION_VALUE, self.DXL_MAXIMUM_POSITION_VALUE]         # Goal position


        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)


        self.dxl_ids = [1,2,3,4,5]
                      #[j1,j2,j3,j4,gripper]

        self.robot_infos={
            "j1":{
                "pos_min":0,
                "origin":0,
                "pos_max":4095,
                "address":1
            },
            "j2":{
                "pos_min":600,
                "origin":1950,
                "pos_max": 3200,
                "address":2
            },
            "j3":{
                "pos_min":0,
                "origin":900,
                "pos_max":1900,
                "address":3
            },
            "j4":{
                "pos_min":1370,
                "origin":2550,
                "pos_max":3950,
                "address":4
            },
            "gripper":{
                "pos_min":0,
                "origin":2120,
                "pos_max":2120,
                "address":5
            },

        }



    def start(self, ):
        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()


        # Set port self.BAUDRATE
        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the self.BAUDRATE")
        else:
            print("Failed to change the self.BAUDRATE")
            print("Press any key to terminate...")
            getch()
            quit()

    def stop(self):
        print("stopping")
        for id in self.dxl_ids:
            print(id)
            self.disable_torque(id)
        # Close port
        self.portHandler.closePort()

    def degree2Dynamixel(self, joint, angle):
        goal = angle*self.ANGLE_DEGREE_COEF+ joint["origin"]
        if goal < joint["pos_min"]:
            print("error value out of limits")
            return(joint["pos_min"])
        elif goal > joint["pos_max"]:
            print("error value out of limits")
            return(joint["pos_max"])
        else:
            return (goal)


    def dynamyxel2degree(self, joint, position):
        return ((position - joint["origin"])/self.ANGLE_DEGREE_COEF)

    def rad2Dynamixel(self, joint, angle):
        goal = angle*self.ANGLE_RAD_COEF+ joint["origin"]
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
        return ((position - joint["origin"])/self.ANGLE_RAD_COEF)
    

    def speedForTable(self, speed):
        return int(abs(speed)/0.229)




    def move_joints(self, j1,j2,j3,j4):
        g1 = int(self.rad2Dynamixel(self.robot_infos["j1"],j1))
        g2 = int(self.rad2Dynamixel(self.robot_infos["j2"],j2))
        g3 = int(self.rad2Dynamixel(self.robot_infos["j3"],j3))
        g4 =  int(self.rad2Dynamixel(self.robot_infos["j4"],j4))

        print(g1,g2,g3,g4)

        self.writeGoalPos(1,g1)
        self.writeGoalPos(2,g2)
        self.writeGoalPos(3,g3)
        self.writeGoalPos(4,g4)
        print("command sent")


    def move_joints_deg(self, j1,j2,j3,j4):
        g1 = int(self.degree2Dynamixel(self.robot_infos["j1"],j1))
        g2 = int(self.degree2Dynamixel(self.robot_infos["j2"],j2))
        g3 = int(self.degree2Dynamixel(self.robot_infos["j3"],j3))
        g4 =  int(self.degree2Dynamixel(self.robot_infos["j4"],j4))

        print(g1,g2,g3,g4)

        self.writeGoalPos(1,g1)
        self.writeGoalPos(2,g2)
        self.writeGoalPos(3,g3)
        self.writeGoalPos(4,g4)
        print("command sent")
        

    def enable_torque(self, id):
        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def disable_torque(self, id):
        # Disable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def write(self, id, goal, nbBytes, tableAddress):
        # Write goal position
        print("id : "+str(id)+ " goal : "+str(goal))
        if(nbBytes == 1):
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, tableAddress, goal)
        elif(nbBytes == 2):
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, tableAddress, goal)
        elif(nbBytes == 4):
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, id, tableAddress, goal)       

        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
    
    def writeGoalPos(self, id, goal):
        self.enable_torque(id)
        self.write( id, goal, self.NBBYTE_GOAL_POSITION, self.ADDR_GOAL_POSITION)

    def writeProfileVelocity(self, id, speed):
        self.write(id, self.speedForTable(speed), self.NBBYTE_PROFILE_VELOCITY, self.ADDR_PROFILE_VELOCITY)

    def writeProfileAcceleration(self, id, acc):
        self.write( id, acc, self.NBBYTE_PROFILE_ACCELERATION, self.ADDR_PROFILE_ACCELERATION)

    def read(self, id):
        # Read present position
            if (self.MY_DXL == 'XL320'): # XL320 uses 2 byte Position Data, Check the size of data in your DYNAMIXEL's control table
                dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, id, self.ADDR_PRESENT_POSITION)
            else:
                dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, id, self.ADDR_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

            print("[ID:%03d] PresPos:%03d" % (id, dxl_present_position))

            return dxl_present_position
    #---------------------------------------------------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        robot = DynamixelArm()
        robot.start()
        while True:
            print("enter your goal position in the form [j1,j2,j3,j4] in rad:")
            goal = literal_eval(input())
            time.sleep(0.5)
            robot.move_joints(goal[0],goal[1],goal[2],goal[3])
            time.sleep(0.5)
        stop()
    except:
        print ('KeyboardInterrupt exception is caught')
        robot.stop()
