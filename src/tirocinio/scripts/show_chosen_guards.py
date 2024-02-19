import cv2
import matplotlib.pyplot as plt

def main():

    img_path = "/home/d-ber/catkin_ws/src/tirocinio/scripts/maps_agp/map_grey_to_black.png"
    img = cv2.imread(img_path)
    data_file_path = "/home/d-ber/catkin_ws/src/tirocinio/optimization/test5/data.dat"
    chosen_guards_file_path = "/home/d-ber/catkin_ws/src/tirocinio/optimization/test5/guardie_scelte.txt"
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
            line = choice_file.readline()
        line = choice_file.readline()
        while line.strip() != ";":
            if line.split()[0] in guardie_scelte:
                posizioni_guardie.append((line.strip().split()[1], line.strip().split()[2]))
            line = choice_file.readline()
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


if __name__ == '__main__':
    main()