import cv2
import numpy as np


class KalmanFilter(): 
    #predifined kalman filter class
    
    def __init__(self):
        print()
    
    #create Kalman filter with 4 stated and 2 measurments
    kf = cv2.KalmanFilter(4,2)
    
    #measurment matarix
    kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
    
    #transition, how the states evolve over time
    kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
    
    
    def predict(self, coordX, coordY):
        measured = np.array([[np.float32(coordX)], [np.float32(coordY)]])
        
        #correct the measurment
        self.kf.correct(measured)
        
        #predict next state, calls the cv2.kalmanfilter method
        predicted = self.kf.predict()
        
        #pos from predicted state
        x, y = predicted[0], predicted[1]
        return x, y
    
#initilize kalman filter
kf = KalmanFilter()

img = cv2.imread("BlueBackground.jpg")

ball_positions = [(50, 100), (100, 100), (150, 100), (200, 100), (250, 100), (300, 100), (350, 100), (400, 100)]
# on the same horizontal line

# For-loop to draw circles for ball positions
for pt in ball_positions:
    cv2.circle(img, pt, 15, (0, 20, 220), -1) #(0, 20, 220) is color

    # Predict the next position using KalmanFilter with two points, i.e frame 1 and frame 2 is used
    predicted = kf.predict(pt[0], pt[1])
    # Convert predicted coordinates to integers before passing to cv2.circle (otherwise parse wrong)
    predicted_int = (int(predicted[0]), int(predicted[1]))
    # Draw a circle for the predicted position
    cv2.circle(img, predicted_int, 15, (0, 255, 0), 3) #green, 4 indicates not a filled circle
    
for i in range(10):
    #predict the next one using the previous predicted predictions 
    predicted= kf.predict(predicted[0], predicted[1])
    predicted_int = (int(predicted[0]), int(predicted[1]))
    cv2.circle(img, predicted_int, 15, (0, 255, 0), 3)
    #print the predicted values (skips the first one since not in for loop)
    print('Predicted values: ', predicted_int)

cv2.imshow("Img", img)
cv2.waitKey(0) #to not close it
