
from util.uri import RMPLAB_Uri
from config import *
from scipy.spatial.transform import Rotation as R
from OneBlock import moveL_delta

TOWER_HEIGHT= 11 
SPEED = 0.5
ACCELERATION = 0.5
BLEND = 0.02
UNIVERSAL_FLOOR = 11
pushed_floor = 6
uri = RMPLAB_Uri()
uri.connect(gripper_calibrate=False)

uri.control.moveL(bay_mid_pose, SPEED, ACCELERATION)
uri.control.moveL(PISTON_INTIAL_POS, SPEED, ACCELERATION)


moveL_delta(uri, 0,0,BLOCK_Z_OFFSET[2]*UNIVERSAL_FLOOR+2, 0, 0, 0,False)

moveL_delta(uri, -0.252,0,0, 0, 0, 0,False)

moveL_delta(uri, 0,0,-(BLOCK_Z_OFFSET[2]*(UNIVERSAL_FLOOR-pushed_floor))-0.085, 0, 0, 0,False)

uri.gripper.close()

moveL_delta(uri, -0.05,0,0, 0, 0, 0,False)

uri.gripper.open()

#Return to PISTON_INTIAL_POS to test the other bricks

moveL_delta(uri, 0,0,BLOCK_Z_OFFSET[2]*2*UNIVERSAL_FLOOR, 0, 0, 0,False)

moveL_delta(uri, 0.252,0,0, 0, 0, 0,False)

uri.control.moveL(PISTON_INTIAL_POS, SPEED, ACCELERATION)

uri.gripper.close()