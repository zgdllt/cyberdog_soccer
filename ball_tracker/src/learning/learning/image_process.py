import cv2
import numpy as np
img= cv2.imread('latest_image.jpg')
# Extract only the green channel (index 1 in BGR format)
hsv= cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
thresh= cv2.inRange(hsv, (30, 70, 0), (70, 255, 255))
# Display the result
# Optionally save the result
cv2.imwrite('thresh.jpg', thresh)