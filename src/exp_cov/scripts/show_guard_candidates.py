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
    guard_position = []
    witness_position = []
    guard_cost = []
    param_end = ";"
    with open(data_file_path, "r") as data_file:
        guard_position_param_start = "param guard_position :="
        line = data_file.readline()
        while line.strip() != guard_position_param_start and line != "":
            line = data_file.readline()
        if line == "":
            raise Exception(f"Start of param guard_position \"{guard_position_param_start}\" not found in file {data_file_path}.")
        line = data_file.readline()
        while line.strip() != param_end and line != "":
            guard_position.append((line.strip().split()[1], line.strip().split()[2]))
            line = data_file.readline()
        if line == "":
            print(f"Warning: End of param guard_position \"{param_end}\" not found in file {data_file_path}.")
    with open(data_file_path, "r") as data_file:
        guard_cost_param_start = "param guard_cost :="
        line = data_file.readline()
        while line.strip() != guard_cost_param_start and line != "":
            line = data_file.readline()
        if line == "":
            raise Exception(f"Start of param guard_cost \"{guard_position_param_start}\" not found in file {data_file_path}.")
        line = data_file.readline()
        while line.strip() != ";" and line != "":
            guard_cost.append(float(line.split()[1]))
            line = data_file.readline()
        if line == "":
            print(f"Warning: End of param guard_cost \"{param_end}\" not found in file {data_file_path}.")
    with open(data_file_path, "r") as data_file:
        witness_position_param_start = "param witness_position :="
        line = data_file.readline()
        while line.strip() != witness_position_param_start and line != "":
            line = data_file.readline()
        if line == "":
            raise Exception(f"Start of param witness_position \"{witness_position_param_start}\" not found in file {data_file_path}.")
        line = data_file.readline()
        while line.strip() != param_end and line != "":
            witness_position.append((line.strip().split()[1], line.strip().split()[2]))
            line = data_file.readline()
        if line == "":
            print(f"Warning: End of param witness_position \"{param_end}\" not found in file {data_file_path}.")
    for i, (x, y) in enumerate(guard_position):
        img = cv2.circle(img, (int(x), int(y)), radius=1, color=(0, 0, 255-(255*(guard_cost[i]))), thickness=-1)
        #img = cv2.circle(img, (int(x), int(y)), radius=1, color=(0, 0, 255-(255*(costi_guardie[i]*4))), thickness=-1) # to boost the color difference
    for (x, y) in witness_position:
        img = cv2.circle(img, (round(float(x)), round(float(y))), radius=1, color=(255, 0, 0), thickness=-1)
    _ = plt.subplot(111), plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB)), plt.title('guards and witnesses')
    plt.show()


if __name__ == '__main__':
    main()