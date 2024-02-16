import cv2
import matplotlib.pyplot as plt

def str_to_float(str):
    str_parts = str.split("/")
    return int(float(str_parts[0])/float(str_parts[1]))

def main():

    img_path = "/home/d-ber/catkin_ws/src/tirocinio/scripts/maps_agp/gt_smoothed_clean.png"
    img = cv2.imread(img_path)
    posizioni = [(44, 114), (128, 192), (140, 222), (158 ,132), (176, 180), (206, 228), (248, 168), (272 ,240), (296 ,144), (338 ,138)]
    for (x, y) in posizioni:
        img = cv2.circle(img, (x,y), radius=2, color=(0, 0, 255), thickness=-1)
    _ = plt.subplot(111), plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB)), plt.title('guards')
    plt.show()

'''
  6  1   6 44 114
202  1 202 128 192
257  1 257 140 222
312  1 312 158 132
376  1 376 176 180
465  1 465 206 228
551  1 551 248 168
616  1 616 272 240
668  1 668 296 144
762  1 762 338 138
'''


if __name__ == '__main__':
    main()