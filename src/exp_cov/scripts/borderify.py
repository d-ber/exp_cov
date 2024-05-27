import os
import numpy as np
import cv2
import sys

if __name__=='__main__':
    if len(sys.argv) == 2 and (sys.argv[1].endswith(".png") or sys.argv[1].endswith(".pgm")):
        img_path = sys.argv[1]
        print(f"proccessing img at {img_path}")
        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        img = img[1:-1,1:-1]
        img = cv2.copyMakeBorder(img, 1, 1, 1, 1, cv2.BORDER_CONSTANT, value=0)
        new_img_filename = os.path.splitext(img_path)[0] + "_bordered" + os.path.splitext(img_path)[1]
        print(new_img_filename)
        cv2.imwrite(new_img_filename, cv2.cvtColor(img, cv2.COLOR_GRAY2RGB))
    else:
        print("Map filename missing. Aborting.")