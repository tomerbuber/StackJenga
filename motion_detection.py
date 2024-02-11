import cv2
import numpy as np

def detect_motion():
        # Initialize webcam
    cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)  # 0 represents the default webcam, change it if necessary

    # We Define ROI (Region of Interest) coordinates and dimensions
    #We want the ROI to include pixels that associate with the top of the jenga pile
    
    #for 6 floors: 
    # roi_x = 387  
    # roi_y = 115  
    # roi_width = 60 
    # roi_height = 60  

    #Adjusted for 12 floors:
    roi_x = 90  # X-coordinate of the top-left corner of the ROI
    roi_y = 115  # Y-coordinate of the top-left corner of the ROI
    roi_width = 60  # Width of the ROI
    roi_height = 60  # Height of the ROI

    # Define the threshold for detecting movement
    threshold = 20000  # Adjust this value based on your needs

    # Capture the initial frame as the reference frame
    ret, reference_frame = cap.read()
    reference_frame_roi = reference_frame[roi_y:roi_y + roi_height, roi_x:roi_x + roi_width]
    reference_frame_gray = cv2.cvtColor(reference_frame_roi, cv2.COLOR_BGR2GRAY)

    # Initialize a flag to track movement
    movement_detected = False

    
    push_counter = 0
    while True:
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
        if diff_sum > threshold:
            cv2.putText(frame, "Movement Detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.rectangle(frame, (roi_x, roi_y), (roi_x + roi_width, roi_y + roi_height), (0, 0, 255), 2)
            movement_detected = True

        # Show the combined frame with the ROI
        cv2.imshow('Live Video with ROI', frame)

        # Update the reference frame for the next iteration (ROI only)
        reference_frame_roi = roi.copy()
        reference_frame_gray = frame_gray.copy()

        # Check for the 'q' key to exit the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

      

    # Release the webcam and close all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

    # If movement was detected, perform some action or return a boolean result
    if movement_detected:
        print("Movement detected")
        return False
    else:
        print("No movement detected")
        return True
    
detect_motion()
