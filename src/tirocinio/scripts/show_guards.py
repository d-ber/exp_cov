import cv2
import matplotlib.pyplot as plt

def str_to_float(str):
    str_parts = str.split("/")
    return int(float(str_parts[0])/float(str_parts[1]))

def main():

    img_path = "/home/aislab/catkin_ws/src/tirocinio/scripts/maps_agp/gt_smoothed.png"
    img = cv2.imread(img_path)
    sol_path = "/home/aislab/catkin_ws/src/tirocinio/scripts/maps_agp/map.sol"
    sol = ""
    with open(sol_path, "r") as sol_file:
        sol_file.readline()
        sol = sol_file.readline()
        guards_num, guards_coords = int(sol.split()[0]), sol.split()[1:]
        for i in range(guards_num):
            x, y = str_to_float(guards_coords[i*2]), str_to_float(guards_coords[i*2+1])
            img = cv2.circle(img, (x,y), radius=2, color=(0, 0, 255), thickness=-1)
        _ = plt.subplot(111), plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB)), plt.title('guards')
        plt.show()




if __name__ == '__main__':
    main()