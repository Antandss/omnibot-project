import cv2
import numpy as np

class KalmanFilter():
    
    def __init__(self):
        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
        self.kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
    
    def predict(self, coordX, coordY):
        measured = np.array([[np.float32(coordX)], [np.float32(coordY)]])
        self.kf.correct(measured)
        predicted = self.kf.predict()
        x, y = predicted[0], predicted[1]
        return x, y

kf = KalmanFilter()

cap = cv2.VideoCapture("IMG_2369.MOV")

prev_points = []

while True:
    ret, frame = cap.read()
    # When the video is finished, break
    if not ret:
        break

    # Convert the frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define the color ranges for red in HSV
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])

    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    # Combine the masks for the two red ranges
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask1, mask2)

    # Find contours in the red mask
    contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw contours on the original frame
    cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)

    # Extract x and y positions of the ball
    for contour in contours:
        moments = cv2.moments(contour)
        if moments["m00"] != 0:
            x = int(moments["m10"] / moments["m00"])
            y = int(moments["m01"] / moments["m00"])
            # Predict the next position using KalmanFilter
            predicted = kf.predict(x, y)
            cv2.circle(frame, (x, y), 10, (255,182,193), -1)
            cv2.circle(frame, (int(predicted[0]), int(predicted[1])), 10, (0, 255, 0), -1)
            prev_points.append((int(predicted[0]), int(predicted[1])))
            
    #to exclude the last point to be drawe which is the predicted
    for i in prev_points[:-1]:
        cv2.circle(frame, i, 10, (255, 0, 0), -1)
        
        
    # for i in range(4):
    #     predicted = kf.predict(predicted[0], predicted[1])
    #     predicted_int = (int(predicted[0]), int(predicted[1]))
    #     cv2.circle(frame, predicted_int, 10, (0, 255, 0), -1)

    cv2.imshow("Frame", frame)
    #cv2.imshow("Red Mask", mask_red)

    # Adjust the delay
    key = cv2.waitKey(300)
    # Break with esc
    if key == 27:
        break
