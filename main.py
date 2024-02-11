
import numpy as np
from util.uri import RMPLAB_Uri
from config import *
from scipy.spatial.transform import Rotation as R


SPEED = 0.5
ACCELERATION = 0.5
def moveL_delta(uri: RMPLAB_Uri, dx, dy, dz, drx=0, dry=0, drz=0):
    tcp = uri.recieve.getActualTCPPose()
    print('move delta - z position: {:.5f}'.format(float(tcp[5])))
    # print('move delta ', tcp)
    tcp[0] = dx; tcp[1] = dy; tcp[2] = dz
    euler = R.from_rotvec(tcp[3:]).as_euler('xyz')
    euler += np.array([drx, dry, drz])
    rot_vec = R.from_euler('xyz', euler).as_rotvec()
    tcp[3:] = rot_vec
    uri.control.moveL(tcp, speed=SPEED, acceleration=ACCELERATION)

def pick_block_from_bay(uri: RMPLAB_Uri, idx: int,floor:int,second_pile_level:bool):
    print(f"current idx:{idx}")
    print(f"current floor:{floor}")
    tcp = uri.recieve.getActualQ()
    print(f"TCP:{tcp}")

    #The bay piles are currently modulo 3
    
    if second_pile_level:
        print(f"updating the floor:")
        floor = floor-1
        print(f"updated floor:{floor}")
    floor = floor%3
    uri.gripper.open()
    
    tcp = uri.recieve.getActualTCPPose()
    # We add the midpoint to make sure the IK will not collide with wall
    # blend = 0.02

    # path_pose1 = [0.3659975095800223, -5.506160059375899e-06, 0.2254876089051707, -2.2989599885299742e-05, -3.1415822233624513, 2.177653165252031e-05, SPEED,ACCELERATION, blend)
    # path_pose2 = [-0.143, -0.51, 0.21, -0.001, 3.12, 0.04, velocity, acceleration, blend_2]
    # path_pose3 = [-0.32, -0.61, 0.31, -0.001, 3.12, 0.04, velocity, acceleration, blend_3]
    # path = [path_pose1, path_pose2, path_pose3]

    # # Send a linear path with blending in between - (currently uses separate script)
    # rtde_c.moveL(path)
    # rtde_c.stopScript()

    uri.control.moveL(bay_mid_pose, SPEED, ACCELERATION)

    uri.control.moveL(bay_top_pose- floor * bay_offset, SPEED, ACCELERATION)     
        

    #Adjusting the pile:

    if idx==5:
        uri.control.moveL((bay_bottom_pose+ 3 * BLOCK_Z_OFFSET) - (floor * bay_offset), SPEED, ACCELERATION)
        q = uri.recieve.getActualQ()
        q[-1] += np.pi / 2
        uri.control.moveJ(q, speed=1.0, acceleration=1.0)

        uri.gripper.close(force=0,speed=5)
        uri.gripper.open()

    if idx==2:
        bay_bottom_pose_higher = bay_bottom_pose.copy()
        bay_bottom_pose_higher[2]+=0.012
        uri.control.moveL((bay_bottom_pose_higher) - (floor * bay_offset), SPEED, ACCELERATION)
        q = uri.recieve.getActualQ()
        q[-1] += np.pi / 2
        uri.control.moveJ(q, speed=1.0, acceleration=1.0)

        uri.gripper.close(force=0,speed=5)
        uri.gripper.open()

    #Lifting the block:


    uri.control.moveL(bay_bottom_pose + idx * BLOCK_Z_OFFSET - floor * bay_offset, SPEED, ACCELERATION)
    uri.gripper.close(speed=1, force=1)
    uri.control.moveL(bay_top_pose- floor * bay_offset, SPEED, ACCELERATION)
    uri.control.moveL(bay_mid_pose, SPEED, ACCELERATION)


def _pick_or_place_block(uri: RMPLAB_Uri, idx: int, pickup,floor:int,tilted:bool,universal_floor:int ): 
    if not tilted :
        block_top = block_zero_top + idx * DELTA_BLOCK_OFFSET
        block_top[2]+= universal_floor*0.016
        block_bottom = block_zero_bottom + idx * DELTA_BLOCK_OFFSET
        block_bottom[2]+= universal_floor*0.016
    #TODO: automate n levels, datstruc that holds cell for TCP each block. code needs to be modular, and add new CONSTS


        uri.control.moveL(block_top, SPEED, ACCELERATION)
        uri.control.moveL(block_bottom, SPEED, ACCELERATION)

        if pickup:
            uri.gripper.close(speed=1, force=1)
        else:

            uri.gripper.open()
        uri.control.moveL(block_top, SPEED, ACCELERATION)


    elif tilted:
       
        block_top = block_zero_top + idx * DELTA_BLOCK_OFFSET
        block_top[2]+= universal_floor*0.016
        block_bottom = block_zero_bottom + idx * DELTA_BLOCK_OFFSET
        block_bottom[2]+= universal_floor*0.016

        # q = uri.recieve.getActualQ()
        # q[-1] += np.pi / 2

        # uri.control.moveJ(q, speed=1.0, acceleration=1.0)
        # #TODO: Create soft version update offset of Z axis 
        tcp = uri.recieve.getActualTCPPose()

        #Adjusting x and z coordiantes
        block_bottom[3:]= tcp[3:].copy()
        
        block_top[3:] = tcp[3:].copy()

       #TODO: add constants

        block_top[0]-=((0.5*0.081)-0.022-(idx*0.024))
        block_bottom[0]-=((0.5*0.081)-0.022-(idx*0.024))
        if idx==0:
            block_top[0]-=0.0055
            block_bottom[0]-=0.0055
            block_top[1]-=0.024
            block_bottom[1]-=0.024
        # elif idx==1:
        #     block_top[1]-=0.048
        #     block_bottom[1]-=0.048
        elif idx == 1:
            block_top[0]-=0.0034
            block_bottom[0]-=0.0034
        elif idx==2:
            block_top[1]+=0.024
            block_bottom[1]+=0.024
            block_top[0]-=0.0039
            block_bottom[0]-=0.0039
        topX = bay_mid_pose[0]-block_top[0]
        topY = bay_mid_pose[1]-block_top[1]
        topZ = bay_mid_pose[2]-block_top[2]

        bottomX = bay_mid_pose[0]-block_bottom[0]
        bottomY = bay_mid_pose[1]-block_bottom[1]
        bottomZ = bay_mid_pose[2]-block_bottom[2]
        print(f"topX:{topX}, topY:{topY}, topZ{topZ}" )
        print(f"bottomX:{bottomX}, bottomY:{bottomY}, bottomZ{bottomZ}")      


        moveL_delta(uri, block_top[0],block_top[1],block_top[2], 0, 0, 1.57079632679)
        moveL_delta(uri, block_bottom[0],block_bottom[1],block_bottom[2], 0, 0, 0)
       
        # block_top[0]=0.235
        # block_top[1]=-0.232

   
        if pickup:
            uri.gripper.close(speed=1, force=1)
        else:

            uri.gripper.open()
        moveL_delta(uri, block_top[0],block_top[1],block_top[2], 0, 0, 0)


def place_block(uri: RMPLAB_Uri, idx: int,floor:int,tilted:bool, universal_floor:int):
    _pick_or_place_block(uri, idx, False,floor,tilted,universal_floor)

def pickup_block(uri: RMPLAB_Uri, idx: int,floor:int,universal_floor:int):
    _pick_or_place_block(uri, idx, True, floor,True,universal_floor)

def adjust_right(uri: RMPLAB_Uri): 
    """
    push rightmost block from right to the middle to adjust it
    """
    right_top = block_zero_top - 1 * DELTA_BLOCK_OFFSET 
    right_bottom = block_zero_bottom - 1 * DELTA_BLOCK_OFFSET
    right_adjust_bottom = block_zero_bottom + 1 * Y_BLOCK_WIDTH
    
    uri.control.moveL(right_top, SPEED, ACCELERATION)
    uri.control.moveL(right_bottom, SPEED, ACCELERATION)
        
    uri.gripper.move_and_wait_for_pos(position=100, speed=30, force=1) 
    # uri.gripper.close(speed=1, )

    uri.control.moveL(right_adjust_bottom, 0.5, 0.5)
    uri.control.moveL(right_top, SPEED, ACCELERATION)

    uri.gripper.open()

def adjust_left(uri: RMPLAB_Uri): 
    """
    push leftmost block from left to the middle to adjust it
    """
    left_top = block_zero_top + 3 * DELTA_BLOCK_OFFSET 
    left_bottom = block_zero_bottom + 3 * DELTA_BLOCK_OFFSET
    left_adjust_bottom = block_zero_bottom + 2 * DELTA_BLOCK_OFFSET - 1 * Y_BLOCK_WIDTH
    
    uri.control.moveL(left_top, SPEED, ACCELERATION)
    uri.control.moveL(left_bottom, SPEED, ACCELERATION)
        
    uri.gripper.move_and_wait_for_pos(position=100, speed=30, force=1) 
    # uri.gripper.close(speed=1, )

    uri.control.moveL(left_adjust_bottom, 0.5, 0.5)
    uri.control.moveL(left_top, SPEED, ACCELERATION)

    uri.gripper.open()



def adjust(uri: RMPLAB_Uri): 
    """
    push leftmost block from left to the middle to adjust it, and rightmost block to the left
    """
    right_top = block_zero_top - 1 * DELTA_BLOCK_OFFSET 
    right_bottom = block_zero_bottom - 1 * DELTA_BLOCK_OFFSET
    right_adjust_bottom = block_zero_bottom + 1 * Y_BLOCK_WIDTH

    left_top = block_zero_top + 3 * DELTA_BLOCK_OFFSET 
    left_bottom = block_zero_bottom + 3 * DELTA_BLOCK_OFFSET
    left_adjust_bottom = block_zero_bottom + 2 * DELTA_BLOCK_OFFSET - 1 * Y_BLOCK_WIDTH

    # partially close fingers for horizontal push
    uri.gripper.move_and_wait_for_pos(position=100, speed=30, force=1) 

    # move right to all blocks
    uri.control.moveL(right_top, SPEED, ACCELERATION)
    uri.control.moveL(right_bottom, SPEED, ACCELERATION)
        
    # gently push left
    uri.control.moveL(right_adjust_bottom, 0.4, 0.4)
    uri.control.moveL(right_top, SPEED, ACCELERATION)
    
    # move left to all blocks
    uri.control.moveL(left_top, SPEED, ACCELERATION)
    uri.control.moveL(left_bottom, SPEED, ACCELERATION)

    # gently push right
    uri.control.moveL(left_adjust_bottom, 0.4, 0.4)
    uri.control.moveL(left_top, SPEED, ACCELERATION)

    uri.gripper.open()

    

if __name__ == "__main__":
    uri = RMPLAB_Uri()
    uri.connect(gripper_calibrate=False)
j=-1
universal_floor = 0
while universal_floor <=11:
    j+=1
    for i in range(3):
        pick_block_from_bay(uri, 5 - i,j,False) #5-> 4-> 3
        place_block(uri, i,j,False,universal_floor)
    j+=1
    universal_floor+=1


    if universal_floor>11:
        break

    for i in range(3):
        pick_block_from_bay(uri, 2 - i,j,True) #2-> 1-> 0
        place_block(uri, i,j,True,universal_floor)
    j-=1
    universal_floor+=1

    if universal_floor<2:
        #TODO: reanalyze/adjust heights and poses for the jenga tower at each floor
        adjust_right(uri)
        adjust_left(uri)
        adjust(uri)
   
