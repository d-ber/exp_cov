import numpy as np
import cv2
#from python_tsp.exact import solve_tsp_dynamic_programming
from python_tsp.heuristics import solve_tsp_local_search, solve_tsp_simulated_annealing


def main():
    dat_path = "/home/d-ber/catkin_ws/src/tirocinio/optimization/test4/data.dat"
    chosen_guards_path = "/home/d-ber/catkin_ws/src/tirocinio/optimization/test4/guardie_scelte.txt"
    distanze = []
    with open(dat_path, "r") as data_file:
        line = data_file.readline()
        while not line.strip().startswith("param nG :="):
            line = data_file.readline()
        nG = int(line.split(":=")[1].strip().split(";")[0])
        line = data_file.readline()
        while not line.strip().startswith("param distanza :"):
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
        while line.strip() != "scelta_guardie [*] :=":
            line = choice_file.readline()
        line = choice_file.readline()
        while line.strip() != ";":
            guardie_scelte.append(float(line.strip().split()[0]))
            line = choice_file.readline()
    pos_guardie = dict()
    with open(chosen_guards_path, "r") as choice_file:
        line = choice_file.readline()
        while line.strip() != "param posizione_guardie :=":
            line = choice_file.readline()
        line = choice_file.readline()
        while line.strip() != ";":
            n_guardia = float(line.strip().split()[0])
            if n_guardia in guardie_scelte:
                pos_guardie[guardie_scelte.index(n_guardia)] = line.strip().split()[1], line.strip().split()[2]
            line = choice_file.readline()
    distanze_scelte = []
    for g1 in range(len(distanze)):
        if g1 in guardie_scelte:
            distanze_g1 = []
            for g2 in range(len(distanze)):
                if g2 in guardie_scelte:
                    distanze_g1.append(distanze[g1][g2])
            distanze_scelte.append(distanze_g1)
    
    #print(f"Solving TSP with {len(distanze_scelte)} nodes")
    
    distance_matrix = np.array(distanze_scelte, dtype=float)
    #permutation, distance = solve_tsp_dynamic_programming(distance_matrix)
    permutation, distance = solve_tsp_simulated_annealing(distance_matrix)
    permutation2, distance2 = solve_tsp_local_search(
        distance_matrix, x0=permutation, perturbation_scheme="ps3"
        )
    #print(permutation2, distance2)

    scale = 0.035888 # scale for map_rgb1
    img = cv2.imread("/home/d-ber/catkin_ws/src/tirocinio/optimization/test4/map_grey_to_black.png")
    image_width = img.shape[1]
    image_height = img.shape[0]
    sizex = img.shape[0]
    sizex = sizex/(1/scale)
    sizey = img.shape[1]
    sizey = sizey/(1/scale)
    size_width = sizey
    size_height = sizex

    for p in permutation2:
        x, y  = pos_guardie[p]
        stage_x = (-size_width/2) + ((size_width/2 - (-size_width/2)) / (image_width - 0)) * (int(x) - 0)
        stage_y = (-size_height/2) + ((size_height/2 - (-size_height/2)) / (image_height - 0)) * ((image_height-int(y)) - 0)
        print(f"{stage_x}, {stage_y}")

if __name__ == '__main__':
    main()