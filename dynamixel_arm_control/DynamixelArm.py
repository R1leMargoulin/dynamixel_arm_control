#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
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
from hardware_infos import *

#********* DYNAMIXEL Model definition *********


class DynamixelArm:
    def __init__(self):

        self.MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430


        self.PROTOCOL_VERSION            = 2.0
        self.DEVICENAME                  = '/dev/ttyACM0'

        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        self.robot_infos= robot_infos
        
        self.giving_orders = False #if it is false, we can read robot position without any conflict



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
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the self.BAUDRATE")
        else:
            print("Failed to change the self.BAUDRATE")
            print("Press any key to terminate...")
            getch()
            quit()

    def stop(self):
        print("stopping")
        self.giving_orders = True
        for j in self.robot_infos.values():
            #print("stopping " + str(j["address"]))
            self.disable_torque(j["address"])
        # Close port
        self.portHandler.closePort()


    def enable_torque(self, id):
        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def disable_torque(self, id):
        # Disable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
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

    def read(self, id, nbBytes, tableAddress):
        # Read present position
            if(nbBytes == 1):
                dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, id, tableAddress)
            elif(nbBytes == 2):
                dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, id, tableAddress)
            elif(nbBytes == 4):
                dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, id, tableAddress)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

            return dxl_present_position

    #---------------------------------------------------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        robot = DynamixelArm()
        robot.start()
        while True:
            time.sleep(0.5)
        stop()
    except:
        print ('KeyboardInterrupt exception is caught')
        robot.stop()
