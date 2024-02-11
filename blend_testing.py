
import cv2
import numpy as np

# Initialize the webcam
cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)

while True:
    ret, frame = cap.read()

    if not ret:
        break

    # Convert the frame to grayscale for better contour detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply a smaller Gaussian blur to reduce noise and improve edge detection
    blurred = cv2.GaussianBlur(gray, (3, 3), 0)

    # Perform edge detection using Canny with lower thresholds
    edges = cv2.Canny(blurred, 30, 90)

    # Find contours in the edge-detected image
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Iterate through detected contours
    for contour in contours:
        # Approximate the contour to a polygon
        epsilon = 0.03 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Check if the polygon has 4 corners (Jenga block)
        if len(approx) == 4:
            # Calculate the center pixel of the Jenga block
            M = cv2.moments(contour)
            if M["m00"] != 0:
                #Add 2 edge detection problem solving to further adjust the calibration limiation:

                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                
                # Draw a smaller red circle around the center pixel
                cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
                # Display the center coordinates inside the circle
                cv2.putText(frame, f'({cX}, {cY})', (cX - 15, cY - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1)

            # Draw smaller red dots on the four corners
            for point in approx:
                x, y = point[0]
                cv2.circle(frame, (x, y), 2, (0, 0, 255), -1)

            # Draw the contour in white with a thinner line
            cv2.drawContours(frame, [contour], -1, (255, 255, 255), 1)

    # Display the processed frame
    cv2.imshow("Jenga Blocks Detection", frame)

    # Exit the loop if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close all windows
cap.release()
cv2.destroyAllWindows()