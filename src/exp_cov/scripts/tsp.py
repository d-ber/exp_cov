import numpy as np
import cv2
import argparse
import os
from python_tsp.exact import solve_tsp_dynamic_programming
#from python_tsp.heuristics import solve_tsp_local_search, solve_tsp_simulated_annealing

def check_positive_float(value):
    try:
        value = float(value)
        if value <= 0:
            raise argparse.ArgumentTypeError("{} is not a positive float".format(value))
    except ValueError:
        raise Exception(f"{value} is not a float")
    return value

def parse_args():

    parser = argparse.ArgumentParser(description='Compute data to run optimization.')
    parser.add_argument('--data', default=os.path.join(os.getcwd(), "data.dat"),
        help="Path to the text data file.", metavar="DATA_PATH")
    parser.add_argument('--guards', default=os.path.join(os.getcwd(), "guards.txt"),
        help="Path to the text guards file.", metavar="GUARDS_PATH")
    parser.add_argument('--img', default=os.path.join(os.getcwd(), "image.png"),
        help="Path to the png image file.", metavar="IMG_PATH")
    parser.add_argument('--scale', default=1, type=check_positive_float,
        help="Image scale.", metavar="SCALE")
    return parser.parse_args()

def main():

    args = parse_args()
    dat_path = args.data
    chosen_guards_path = args.guards
    distanze = []
    with open(dat_path, "r") as data_file:
        line = data_file.readline()
        while not line.strip().startswith("param nG :="):
            line = data_file.readline()
        # nG = int(line.split(":=")[1].strip().split(";")[0])
        line = data_file.readline()
        while not line.strip().startswith("param distance :"):
            line = data_file.readline()
        line = data_file.readline()
        while line.strip() != ";":
            pieces = line.split()
            _, distanze_g = pieces[0], pieces[1:]
            distanze.append([float(x) for x in distanze_g])
            line = data_file.readline()
    guardie_scelte = []
    with open(chosen_guards_path, "r") as choice_file:
        line = choice_file.readline()
        while line.strip() != "guard_choice [*] :=":
            line = choice_file.readline()
        line = choice_file.readline()
        while line.strip() != ";":
            guardie_scelte.append(float(line.strip().split()[0]))
            line = choice_file.readline()
    pos_guardie = dict()
    with open(dat_path, "r") as data_file:
        line = data_file.readline()
        while line.strip() != "param guard_position :=":
            line = data_file.readline()
        line = data_file.readline()
        while line.strip() != ";":
            n_guardia = float(line.strip().split()[0])
            if n_guardia in guardie_scelte:
                pos_guardie[guardie_scelte.index(n_guardia)] = line.strip().split()[1], line.strip().split()[2]
            line = data_file.readline()
    distanze_scelte = []
    for g1 in range(len(distanze)):
        if g1 in guardie_scelte:
            distanze_g1 = []
            for g2 in range(len(distanze)):
                if g2 in guardie_scelte:
                    distanze_g1.append(distanze[g1][g2])
            distanze_scelte.append(distanze_g1)
        
    distance_matrix = np.array(distanze_scelte, dtype=float)
    permutation, _ = solve_tsp_dynamic_programming(distance_matrix)
    #permutation, _ = solve_tsp_simulated_annealing(distance_matrix)
    #permutation2, _ = solve_tsp_local_search(
    #    distance_matrix, x0=permutation, perturbation_scheme="ps3"
    #    )

    scale = args.scale
    img = cv2.imread(args.img)
    image_width = img.shape[1]
    image_height = img.shape[0]
    sizex = img.shape[0]
    sizex = sizex/(1/scale)
    sizey = img.shape[1]
    sizey = sizey/(1/scale)
    size_width = sizey
    size_height = sizex

    for p in permutation:
        x, y  = pos_guardie[p]
        stage_x = (-size_width/2) + ((size_width/2 - (-size_width/2)) / (image_width - 0)) * (int(x) - 0)
        stage_y = (-size_height/2) + ((size_height/2 - (-size_height/2)) / (image_height - 0)) * ((image_height-int(y)) - 0)
        print(f"{stage_x}, {stage_y}")

if __name__ == '__main__':
    main()