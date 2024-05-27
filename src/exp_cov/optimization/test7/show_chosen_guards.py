import cv2
import matplotlib.pyplot as plt

def main():

    img_path = "/home/aislab/catkin_ws/src/exp_cov/optimization/test7/tri.png"
    img = cv2.imread(img_path)
    chosen_guards_file_path = "/home/aislab/catkin_ws/src/exp_cov/optimization/test7/guardie_scelte.txt"
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
    for (x, y) in posizioni_guardie:
        img = cv2.circle(img, (int(x), int(y)), radius=1, color=(0, 0, 255), thickness=-1)
    _ = plt.subplot(111), plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB)), plt.title('Chosen Guards')
    plt.show()


if __name__ == '__main__':
    main()