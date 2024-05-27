import cv2
import matplotlib.pyplot as plt

def main():

    img_path = "/home/d-ber/catkin_ws/src/exp_cov/scripts/maps_agp/map_grey_to_black.png"
    img = cv2.imread(img_path)
    data_file_path = "/home/d-ber/catkin_ws/src/exp_cov/optimization/test4/data.dat"
    chosen_guards_file_path = "/home/d-ber/catkin_ws/src/exp_cov/optimization/test4/guardie_scelte.txt"
    posizioni_guardie = []
    with open(chosen_guards_file_path, "r") as data_file:
        guardie_scelte = []
        line = data_file.readline()
        while line.strip() != "scelta_guardie [*] :=":
            line = data_file.readline()
        line = data_file.readline()
        while line.strip() != ";":
            guardie_scelte.append(line.strip().split()[0])
            line = data_file.readline()
        while line.strip() != "param posizione_guardie :=":
            line = data_file.readline()
        line = data_file.readline()
        while line.strip() != ";":
            if line.split()[0] in guardie_scelte:
                posizioni_guardie.append((line.strip().split()[1], line.strip().split()[2]))
            line = data_file.readline()
    
    scale = 0.035888 # scale for map_rgb1
    image_width = img.shape[1]
    image_height = img.shape[0]
    sizex = img.shape[0]
    sizex = sizex/(1/scale)
    sizey = img.shape[1]
    sizey = sizey/(1/scale)
    size_width = sizey
    size_height = sizex

    with open("posizioni.txt", "w") as file_posizioni:
        for (x, y) in posizioni_guardie:
            # Convert to stage coordinates
            stage_x = (-size_width/2) + ((size_width/2 - (-size_width/2)) / (image_width - 0)) * (int(x) - 0)
            stage_y = (-size_height/2) + ((size_height/2 - (-size_height/2)) / (image_height - 0)) * ((image_height-int(y)) - 0)
            file_posizioni.write(f"{stage_x}, {stage_y}\n")
            img = cv2.circle(img, (int(x), int(y)), radius=1, color=(0, 0, 255), thickness=-1)
        _ = plt.subplot(111), plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB)), plt.title('Chosen Guards')
        plt.show()


if __name__ == '__main__':
    main()