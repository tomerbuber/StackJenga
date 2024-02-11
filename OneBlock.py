
import numpy as np
import cv2
from util.uri import RMPLAB_Uri
from config import *
from scipy.spatial.transform import Rotation as R
import time

#Number of floors in the tower -1 (COUNTING FROM 0)
SPEED = 0.5
ACCELERATION = 0.5
BLEND = 0.02
UNIVERSAL_FLOOR = 5



def moveL_delta(uri: RMPLAB_Uri, dx, dy, dz, drx=0, dry=0, drz=0, reset_XYZ = False):
    tcp = uri.recieve.getActualTCPPose()
    # print('move delta ', tcp)

    #To decide if we add delta dx,dy,dz to the  current TCP coordiantes or change them completely- the rotvecs are always moved by deltas.
    if reset_XYZ:
        tcp[0] = dx; tcp[1] = dy; tcp[2] = dz
    else:
         tcp[0] += dx; tcp[1] += dy; tcp[2] += dz

    euler = R.from_rotvec(tcp[3:]).as_euler('xyz')
    euler += np.array([drx, dry, drz])
    rot_vec = R.from_euler('xyz', euler).as_rotvec()
    tcp[3:] = rot_vec
    uri.control.moveL(tcp, speed=SPEED, acceleration=ACCELERATION)


def pick_piston_from_bay(uri: RMPLAB_Uri):
    uri.gripper.open()
    uri.control.moveL(bay_mid_pose, SPEED, ACCELERATION)
    uri.control.moveL(bay_top_pose, SPEED, ACCELERATION)


    bay_bottom_pose[2]+=0.012
    uri.control.moveL(bay_bottom_pose, SPEED, ACCELERATION)
    uri.gripper.close(force=0,speed=5)
    uri.gripper.open()
    q = uri.recieve.getActualQ()
    q[-1] += np.pi / 2
    uri.control.moveJ(q, speed=1.0, acceleration=1.0)


    # uri.control.moveL(bay_bottom_pose, SPEED, ACCELERATION)
    # uri.gripper.close(force=0,speed=5)
    # uri.gripper.open()

    # uri.control.moveJ(q, speed=1.0, acceleration=1.0)
    moveL_delta(uri, 0.0185,0,0,0,0,0)
    uri.gripper.close(force=0,speed=5)
    uri.control.moveL(bay_mid_pose, SPEED, ACCELERATION)



# Initialize webcam
cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)  # 0 represents the default webcam, change it if necessary

# We Define ROI (Region of Interest) coordinates and dimensions
#We want the ROI to include pixels that associate with the top of the jenga pile

roi_x = 90  # X-coordinate of the top-left corner of the ROI
roi_y = 115  # Y-coordinate of the top-left corner of the ROI
roi_width = 60  # Width of the ROI
roi_height = 60  # Height of the ROI

# Define the threshold for detecting movement
threshold = 10000  # Adjust this value based on your needs

# Capture the initial frame as the reference frame
ret, reference_frame = cap.read()
reference_frame_roi = reference_frame[roi_y:roi_y + roi_height, roi_x:roi_x + roi_width]
reference_frame_gray = cv2.cvtColor(reference_frame_roi, cv2.COLOR_BGR2GRAY)
#We start from the first floor, 
uri = RMPLAB_Uri()
uri.connect(gripper_calibrate=False)
# Initialize a flag to track movement
pick_piston_from_bay(uri)

#Move uri to be at ready position infront of middle brick of floor 0

uri.control.moveL(PISTON_INTIAL_POS, SPEED, ACCELERATION)


movement_detected = False
pushed_floor = 0
push_index = 0
new_floor = False
camera_flag = True

# ret, reference_frame = cap.read()
# reference_frame_roi = reference_frame[roi_y:roi_y + roi_height, roi_x:roi_x + roi_width]
# reference_frame_gray = cv2.cvtColor(reference_frame_roi, cv2.COLOR_BGR2GRAY)

while pushed_floor<= UNIVERSAL_FLOOR-1:
    

    print(f"currently trying to push middle block of floor: {pushed_floor}")
    movement_detected = False
    # Capture the current frame
    ret, frame = cap.read()
    if not ret:
        break

    # Extract the ROI from the current frame
    roi = frame[roi_y:roi_y + roi_height, roi_x:roi_x + roi_width]

    # Convert both frames to grayscale
    frame_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

    # Calculate the absolute difference between the reference frame and the current frame in the ROI
    frame_diff = cv2.absdiff(reference_frame_gray, frame_gray)

    # Calculate the sum of pixel differences within the ROI
    diff_sum = np.sum(frame_diff)

    # If the sum exceeds the threshold, movement is detected
    if diff_sum > threshold and camera_flag == True:
        cv2.putText(frame, "Movement Detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.rectangle(frame, (roi_x, roi_y), (roi_x + roi_width, roi_y + roi_height), (0, 0, 255), 2)
        print(f"Camera detected movement, Are we looking for it? {camera_flag}")
        movement_detected = True

    # Show the combined frame with the ROI
    cv2.imshow('Live Video with ROI', frame)
    
    # Update the reference frame for the next iteration (ROI only)
    reference_frame_roi = roi.copy()
    reference_frame_gray = frame_gray.copy()

    # Check for the 'q' key to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    
    if not movement_detected and camera_flag == True:
        new_floor = False
        #If movement detected wasnt set to True, we can continue to move the brick, until push_index reached 4
        print(f"Movement was NOT detected, Trying to push the block further!, push number: {push_index}")
        if push_index == 0:
                #First push :
                
            moveL_delta(uri, -0.020,0,0, 0, 0, 0,False)
            moveL_delta(uri, 0.020,0,0, 0, 0, 0,False)
            push_index+=1

        elif push_index ==1:

        # Second push:
            moveL_delta(uri, -0.028,0,0, 0, 0, 0,False)
            moveL_delta(uri, 0.028,0,0, 0, 0, 0,False)
            push_index+=1

        elif push_index ==2:
            # Third push:
            moveL_delta(uri, -0.033,0,0, 0, 0, 0,False)
            moveL_delta(uri, 0.033,0,0, 0, 0, 0,False)
            push_index+=1

        elif push_index ==3:
            # Fourth push:
            moveL_delta(uri, -0.05,0,0, 0, 0, 0,False)
            moveL_delta(uri, 0.05,0,0, 0, 0, 0,False)
            push_index+=1

        if push_index > 3:
            camera_flag= False
            

        if camera_flag == False:

                #Pull the jenga brick from the other side:

            uri.control.moveL(PISTON_INTIAL_POS, SPEED, ACCELERATION)

            uri.gripper.open()

    
            moveL_delta(uri, 0,0,BLOCK_Z_OFFSET[2]*2*UNIVERSAL_FLOOR, 0, 0, 0,False)

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
            
            camera_flag = True
            pushed_floor+=2 
            push_index = 0
            new_floor = True
            moveL_delta(uri, 0,0,BLOCK_Z_OFFSET[2]*pushed_floor, 0, 0, 0,False)
            
            ret, reference_frame = cap.read()
            reference_frame_roi = reference_frame[roi_y:roi_y + roi_height, roi_x:roi_x + roi_width]
            reference_frame_gray = cv2.cvtColor(reference_frame_roi, cv2.COLOR_BGR2GRAY)
            reference_frame_roi = roi.copy()
            reference_frame_gray = frame_gray.copy()
            # Extract the ROI from the current frame
            roi = frame[roi_y:roi_y + roi_height, roi_x:roi_x + roi_width]


           
            # Convert both frames to grayscale
            frame_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

            # Calculate the absolute difference between the reference frame and the current frame in the ROI
            frame_diff = cv2.absdiff(reference_frame_gray, frame_gray)

            # Calculate the sum of pixel differences within the ROI
            diff_sum = np.sum(frame_diff)


    elif movement_detected:
        print("Movement was detected, moving to the next floor!")
        pushed_floor+=2 
        push_index = 0
        moveL_delta(uri, 0,0,BLOCK_Z_OFFSET[2]*2, 0, 0, 0,False)
        new_floor = True

        ret, reference_frame = cap.read()
        reference_frame_roi = reference_frame[roi_y:roi_y + roi_height, roi_x:roi_x + roi_width]
        reference_frame_gray = cv2.cvtColor(reference_frame_roi, cv2.COLOR_BGR2GRAY)
        reference_frame_roi = roi.copy()
        reference_frame_gray = frame_gray.copy()
        # Extract the ROI from the current frame
        roi = frame[roi_y:roi_y + roi_height, roi_x:roi_x + roi_width]


        
        # Convert both frames to grayscale
        frame_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # Calculate the absolute difference between the reference frame and the current frame in the ROI
        frame_diff = cv2.absdiff(reference_frame_gray, frame_gray)

        # Calculate the sum of pixel differences within the ROI
        diff_sum = np.sum(frame_diff)


    # Release the webcam and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
uri.control.moveL(PISTON_INTIAL_POS, SPEED, ACCELERATION)
uri.gripper.open()
moveL_delta(uri, 0,0,BLOCK_Z_OFFSET[2]*2, 0, 0, 0,False)


