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
    parser.add_argument('--guards', default=os.path.join(os.getcwd(), "guards.txt"),
        help="Path to the text guards file.", metavar="GUARDS_PATH")
    return parser.parse_args()

def read_param(file, param_start, param_end, param_name, value_read, filter):
    with open(file, "r") as file:
        param_values = []
        line = file.readline()
        param_start = param_start
        while line.strip() != param_start and line != "":
            line = file.readline()
        if line == "":
            raise Exception(f"Start of param param_name \"{param_start}\" not found in file {file}.")
        line = file.readline()
        while line.strip() != param_end and line != "":
            if filter(line):
                param_values.append(value_read(line))
            line = file.readline()
        if line == "":
            print(f"Warning: End of param param_name \"{param_end}\" not found in file {file}.")
        return param_values

def main():

    args = parse_args()
    img_path = args.img
    img = cv2.imread(img_path)
    data_file_path = args.data
    chosen_guards_file_path = args.guards
    guard_position = []
    guard_choice = []
    witness_position = []
    guard_cost = []
    param_end = ";"
    if chosen_guards_file_path != "":
        guard_choice = read_param(chosen_guards_file_path, "guard_choice [*] :=", param_end, "guard_choice", lambda line: line.strip().split()[0], lambda line: True)
        guard_position = read_param(data_file_path, "param guard_position :=", param_end, "guard_position", lambda line: (line.strip().split()[1], line.strip().split()[2]), lambda line: line.split()[0] in guard_choice)
        witness_position = read_param(data_file_path, "param witness_position :=", param_end, "witness_position", lambda line: (line.strip().split()[1], line.strip().split()[2]), lambda line: True)

        for (x, y) in guard_position:
            img = cv2.circle(img, (int(x), int(y)), radius=1, color=(0, 0, 255), thickness=-1)
        for (x, y) in witness_position:
            img = cv2.circle(img, (round(float(x)), round(float(y))), radius=1, color=(255, 0, 0), thickness=-1)
        _ = plt.subplot(111), plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB)), plt.title('Chosen Guards')
        plt.show()
    else:
        guard_cost = read_param(data_file_path, "param guard_cost :=", param_end, "guard_cost", lambda line: float(line.split()[1]), lambda line: True)
        guard_position = read_param(data_file_path, "param guard_position :=", param_end, "guard_position", lambda line: (line.strip().split()[1], line.strip().split()[2]), lambda line: True)
        witness_position = read_param(data_file_path, "param witness_position :=", param_end, "witness_position", lambda line: (line.strip().split()[1], line.strip().split()[2]), lambda line: True)
        
        for i, (x, y) in enumerate(guard_position):
            img = cv2.circle(img, (int(x), int(y)), radius=1, color=(0, 0, 255-(255*(guard_cost[i]))), thickness=-1)
            #img = cv2.circle(img, (int(x), int(y)), radius=1, color=(0, 0, 255-(255*(costi_guardie[i]*4))), thickness=-1) # to boost the color difference
        for (x, y) in witness_position:
            img = cv2.circle(img, (round(float(x)), round(float(y))), radius=1, color=(255, 0, 0), thickness=-1)
        _ = plt.subplot(111), plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB)), plt.title('guards and witnesses')
        plt.show()

if __name__ == '__main__':
    main()