import cv2
import yaml

borderType = cv2.BORDER_CONSTANT
grey = [205]

small = cv2.imread("pad/small.png", cv2.IMREAD_GRAYSCALE)
small_height, small_width = small.shape[:2]
small_info = None
with open('pad/small.yaml', 'r') as file:
    small_info = yaml.safe_load(file)

big = cv2.imread("pad/big.png", cv2.IMREAD_GRAYSCALE)
big_height, big_width = big.shape[:2]
big_info = None
with open('pad/big.yaml', 'r') as file:
    big_info = yaml.safe_load(file)

# TODO?: check vari: stessa risoluzione...

bottom = round(abs(big_info["origin"][1] - small_info["origin"][1]) * (1 / big_info["resolution"]))
top = round(abs(big_height - (small_height + bottom)))
left = round(abs(big_info["origin"][0] - small_info["origin"][0]) * (1 / big_info["resolution"]))
right = round(abs(big_width - (small_width + left)))

#print(top, bottom, left, right)

padded = cv2.copyMakeBorder(small, top, bottom, left, right, borderType, None, grey)

cv2.imwrite("pad/padded.png", padded)