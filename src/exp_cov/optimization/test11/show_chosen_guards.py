import cv2
import matplotlib.pyplot as plt

def main():

    img_path = "/home/d-ber/catkin_ws/src/exp_cov/optimization/test11/tri.png"
    img = cv2.imread(img_path)

    image_width_small = img.shape[1]
    image_height_small = img.shape[0]

    data_file_path = "/home/d-ber/catkin_ws/src/exp_cov/optimization/test11/data.dat"
    chosen_guards_file_path = "/home/d-ber/catkin_ws/src/exp_cov/optimization/test11/guardie_scelte.txt"
    posizioni_guardie = []
    with open(chosen_guards_file_path, "r") as choice_file:
        guardie_scelte = []
        line = choice_file.readline()
        while line.strip() != "scelta_guardie [*] :=":
            line = choice_file.readline()
        line = choice_file.readline()
        while line.strip() != ";":
            guardie_scelte.append(line.strip().split()[0])
            line = choice_file.readline()
    with open(data_file_path, "r") as data_file:
        while line.strip() != "param posizione_guardie :=":
            line = data_file.readline()
        line = data_file.readline()
        while line.strip() != ";":
            if line.split()[0] in guardie_scelte:
                posizioni_guardie.append((line.strip().split()[1], line.strip().split()[2]))
            line = data_file.readline()
    temp = posizioni_guardie[0]
    posizioni_guardie[0] = posizioni_guardie[1]
    posizioni_guardie[1] = temp
    print(posizioni_guardie)
    posizioni_testimoni = []
    with open(data_file_path, "r") as data_file:
        line = data_file.readline()
        while line.strip() != "param posizione_testimoni :=":
            line = data_file.readline()
        line = data_file.readline()
        while line.strip() != ";":
            posizioni_testimoni.append((line.strip().split()[1], line.strip().split()[2]))
            line = data_file.readline()
    for (x, y) in posizioni_guardie:
        img = cv2.circle(img, (int(x), int(y)), radius=1, color=(0, 0, 255), thickness=-1)
    for (x, y) in posizioni_testimoni:
        img = cv2.circle(img, (round(float(x)), round(float(y))), radius=1, color=(255, 0, 0), thickness=-1)
    _ = plt.subplot(111), plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB)), plt.title('Chosen Guards')
    plt.show()

    stage_image = "/home/d-ber/catkin_ws/src/exp_cov/maps_rgb_lab/map1/map1_rgb.png"
    image = cv2.imread(stage_image, cv2.IMREAD_GRAYSCALE)
    scale = 0.035888
    sizex = image.shape[0]
    sizex = sizex/(1/scale)
    sizey = image.shape[1]
    sizey = sizey/(1/scale)
    image_width = image.shape[1]
    image_height = image.shape[0]
    size_width = sizey
    size_height = sizex
    
    for i in range(len(guardie_scelte)):
        x, y = int(posizioni_guardie[i][0]), int(posizioni_guardie[i][1])
        x = (x * image_width) / image_width_small
        y = (y * image_height) / image_height_small
        x = (-size_width/2) + ((size_width/2 - (-size_width/2)) / (image_width - 0)) * (x - 0)
        y = (-size_height/2) + ((size_height/2 - (-size_height/2)) / (image_height - 0)) * ((image_height-y) - 0)
        print(f"{guard_scelte_ordine.index(i)}: {x}, {y}")
    cv2.imwrite("tesi_scelte_ordinate.png", img)


if __name__ == '__main__':
    main()