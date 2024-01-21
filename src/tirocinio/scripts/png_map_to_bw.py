import cv2
import matplotlib.pyplot as plt

img_path = "/home/d-ber/Desktop/area4_floor_0.png"
new_path = "/home/d-ber/Desktop/area4_floor_0_b&w.png"
img = cv2.imread(img_path)
img = cv2.threshold(img, 254, 255, cv2.THRESH_BINARY)[1]
cv2.imwrite(new_path, img)