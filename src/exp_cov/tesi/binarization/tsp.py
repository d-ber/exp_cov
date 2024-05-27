import numpy as np
from python_tsp.exact import solve_tsp_dynamic_programming

distance_matrix = np.array([
[0, 113, 77, 43, 53, 61, 93, 130, 129],
[113, 0, 38, 134, 61, 101, 109, 146, 145],
[77, 38, 0, 98, 25, 65, 73, 110, 109],
[43, 134, 98, 0, 74, 82, 114, 151, 150],
[53, 61, 25, 74, 0, 41, 49, 86, 85],
[61, 101, 65, 82, 41, 0, 81, 118, 117],
[93, 109, 73, 114, 49, 81, 0, 49, 37],
[130, 146, 110, 151, 86, 118, 49, 0, 71],
[129, 145, 109, 150, 85, 117, 37, 71, 0]
])
distance_matrix[:, 0] = 0
permutation, distance = solve_tsp_dynamic_programming(distance_matrix)
print(permutation, distance)