import cv2
import matplotlib.pyplot as plt
import argparse
import os

def parse_args():

    parser = argparse.ArgumentParser(description='Compute data to run optimization.')
    parser.add_argument('--img', default=os.path.join(os.getcwd(), "image.png"),
        help="Path to the png image file.", metavar="IMG_PATH")
    parser.add_argument('--data', default=os.path.join(os.getcwd(), "data.dat"),
        help="Path to the text data file.", metavar="DATA_PATH")
    return parser.parse_args()

def main():

    args = parse_args()

    img_path = args.img
    img = cv2.imread(img_path)
    data_file_path = args.data
    posizioni_guardie = []
    with open(data_file_path, "r") as data_file:
        line = data_file.readline()
        while line.strip() != "param guard_position :=":
            line = data_file.readline()
        line = data_file.readline()
        while line.strip() != ";":
            posizioni_guardie.append((line.split()[1], line.split()[2]))
            line = data_file.readline()
    costi_guardie = []
    with open(data_file_path, "r") as data_file:
        line = data_file.readline()
        while line.strip() != "param guard_cost :=":
            line = data_file.readline()
        line = data_file.readline()
        while line.strip() != ";":
            costi_guardie.append(float(line.split()[1]))
            line = data_file.readline()
    posizioni_testimoni = []
    with open(data_file_path, "r") as data_file:
        line = data_file.readline()
        while line.strip() != "param witness_position :=":
            line = data_file.readline()
        line = data_file.readline()
        while line.strip() != ";":
            posizioni_testimoni.append((line.split()[1], line.split()[2]))
            line = data_file.readline()
    for i, (x, y) in enumerate(posizioni_guardie):
        img = cv2.circle(img, (int(x), int(y)), radius=1, color=(0, 0, 255-(255*(costi_guardie[i]))), thickness=-1)
        #img = cv2.circle(img, (int(x), int(y)), radius=1, color=(0, 0, 255-(255*(costi_guardie[i]*4))), thickness=-1) # to boost the color difference
    for (x, y) in posizioni_testimoni:
        img = cv2.circle(img, (round(float(x)), round(float(y))), radius=1, color=(255, 0, 0), thickness=-1)
    _ = plt.subplot(111), plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB)), plt.title('guards and witnesses')
    plt.show()


if __name__ == '__main__':
    main()