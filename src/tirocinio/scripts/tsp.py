import numpy as np
#from python_tsp.exact import solve_tsp_dynamic_programming
from python_tsp.heuristics import solve_tsp_local_search, solve_tsp_simulated_annealing


def main():
    dat_path = "/home/aislab/catkin_ws/src/tirocinio/optimization/test7/data.dat"
    chosen_guards_path = "/home/aislab/catkin_ws/src/tirocinio/optimization/test7/guardie_scelte.txt"
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
    distanze_scelte = []
    for g1 in range(len(distanze)):
        if g1 in guardie_scelte:
            distanze_g1 = []
            for g2 in range(len(distanze)):
                if g2 in guardie_scelte:
                    distanze_g1.append(distanze[g1][g2])
            distanze_scelte.append(distanze_g1)
    
    print(f"Solving TSP with {len(distanze_scelte)} nodes")
    
    distance_matrix = np.array(distanze_scelte, dtype=float)
    #permutation, distance = solve_tsp_dynamic_programming(distance_matrix)
    permutation, distance = solve_tsp_simulated_annealing(distance_matrix)
    permutation2, distance2 = solve_tsp_local_search(
        distance_matrix, x0=permutation, perturbation_scheme="ps3"
        )
    print(permutation2, distance2)

if __name__ == '__main__':
    main()