import cv2
import matplotlib.pyplot as plt

def main():

    map_path = "/home/d-ber/catkin_ws/src/tirocinio/scripts/maps_agp/map.png"
    img_map = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
    #img_map = cv2.threshold(img_map, 254, 255, cv2.THRESH_BINARY)[1]
    floorplan_path = "/home/d-ber/catkin_ws/src/tirocinio/scripts/maps_agp/gt_floorplan.png"
    img_floorplan = cv2.imread(floorplan_path, cv2.IMREAD_GRAYSCALE)
    height, width = img_floorplan.shape
    points = (width, height)
    new_map1 = cv2.resize(img_map, points, interpolation= cv2.INTER_NEAREST)
    new_map2 = cv2.resize(img_map, points, interpolation= cv2.INTER_NEAREST_EXACT)

    DEBUG = False
    if DEBUG:
        _ = plt.subplot(221), plt.imshow(cv2.cvtColor(img_map, cv2.COLOR_BGR2RGB)), plt.title('original map')
        _ = plt.subplot(222), plt.imshow(cv2.cvtColor(img_floorplan, cv2.COLOR_BGR2RGB)), plt.title('floorplan')
        _ = plt.subplot(223), plt.imshow(cv2.cvtColor(new_map1, cv2.COLOR_BGR2RGB)), plt.title('INTER_NEAREST interpolated')
        _ = plt.subplot(224), plt.imshow(cv2.cvtColor(new_map2, cv2.COLOR_BGR2RGB)), plt.title('INTER_NEAREST_EXACT interpolated')
        plt.show()
    cv2.imwrite("INTER_NEAREST.png", new_map1)
    cv2.imwrite("INTER_NEAREST_EXACT.png", new_map2)

if __name__ == "__main__":
    main()