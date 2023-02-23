from math import pi
# Control table address

ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132
ADDR_PROFILE_VELOCITY       = 112
ADDR_PROFILE_ACCELERATION   = 108

#controle table data bytes number
NBBYTE_GOAL_POSITION        = 4
NBBYTE_PROFILE_VELOCITY     = 4
NBBYTE_PROFILE_ACCELERATION = 4
NBBYTE_PRESENT_POSITION = 4

#values
DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
BAUDRATE                    = 57600
ANGLE_DEGREE_COEF = 4096/360
ANGLE_RAD_COEF = 4096/(2*pi)
TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position



dxl_ids = [1,2,3,4,5]
        #[j1,j2,j3,j4,gripper]

robot_infos={
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