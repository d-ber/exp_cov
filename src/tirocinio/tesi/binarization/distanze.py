import cv2
import shapely
import shapely.validation
import matplotlib.pyplot as plt
import concurrent.futures
import progressbar
#import pyomo.environ as pyo
import numpy as np
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
import math


def print_simple_param(name, value):
    print(f"param {name} := {value};")

def print_vector_param(name, pairs):
    print(f"param {name} := ")
    for (index, value) in pairs:
        print(f"{index} {value}")
    print(";")

def print_bidimensional_param(name, rows, cols, vals):
    #print(f"DEBUG: name:{name} rows:{rows} cols:{cols} vals[0]:{vals[0]}")
    print(f"param {name} : ", end="")
    for col in range(1, cols+1):
        print(f" {col} ", end="")
    print(f" := ")
    for row in range(1, rows+1):
        print(f"{row} ", end="")
        for col in range(0, cols):
            print(f"{vals[row-1][col]} ", end="")
        print("")
    print(";")

def copertura_w(copertura_w_data):
    MAX_VISIBILITY_RANGE = 10000
    i, poly, w, guards = copertura_w_data
    copertura_w = []
    for j, g in enumerate(guards):
        seg = shapely.LineString([g, w])
        if poly.contains(seg) and seg.length < MAX_VISIBILITY_RANGE:
            copertura_w.insert(j, 1)
        else:
            copertura_w.insert(j, 0)
    return (i, copertura_w)

def distanza_gg(distanze_gg_data):
    dis_gg = []
    i, g1, guards, pathfinding_matrix = distanze_gg_data
    finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
    g1_x, g1_y = g1 
    for i2, g2 in enumerate(guards):
        pathfinding_grid = Grid(matrix=pathfinding_matrix)
        start = pathfinding_grid.node(g1_x, g1_y)
        g2_x, g2_y = g2
        end = pathfinding_grid.node(g2_x, g2_y)
        path, _ = finder.find_path(start, end, pathfinding_grid)
        dis_gg.insert(i2, len(path))
    return (i, dis_gg)

def min_distance_to_holes(poly_with_holes, point):
    if len(poly_with_holes.interiors) == 0:
        return -1 
    return min([hole.distance(point) for hole in poly_with_holes.interiors])

def print_dat(poly, pathfinding_matrix):
    GUARD_MAXIMUM_NUMBER = 300
    MIN_DISTANCE_TO_POLY = 5
    print("\ndata;")

    '''
    23  50 48
    32  66 96
    71 86 56
    108 98 124
    127 110 68
    128 126 100
    130 158 64
    163 178 112
    206 194 48
    '''

    guards = [(66, 96), (50, 48), (86, 56), (98, 124), (110, 68), (126, 100), (158, 64), (178, 112), (194, 48)]
    nG = len(guards)    
    distanze = []
    with concurrent.futures.ProcessPoolExecutor(max_workers=20) as executor:
        distanze_gg_data = [(i, g1, guards, pathfinding_matrix) for i, g1 in enumerate(guards)]
        for i, dis_gg in executor.map(distanza_gg, distanze_gg_data):
            distanze.insert(i, dis_gg)
    print_bidimensional_param("distanza", nG, nG, distanze)

    print("\nend;")

    # Save guard location outside problem data since it's of no use (distances are precalculated)
    print_vector_param("posizione_guardie", [(i+1, (f"{x} {y}")) for (i, (x,y)) in enumerate(guards)])


def main():

    img_path = "/home/d-ber/catkin_ws/src/tirocinio/tesi/binarization/tri.png"
    MIN_HOLE_AREA = 10
    DEBUG_HOLES = False
    DEBUG_CONTOUR = False 
    img = cv2.imread(img_path)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    contours, _ = cv2.findContours(img, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    max_contour = max(contours, key = lambda cnt: cv2.contourArea(cnt))
    if DEBUG_CONTOUR:
        debug = np.zeros_like(img)
        cv2.drawContours(debug, [max_contour], -1, (255,255,255), cv2.FILLED)
        _ = plt.subplot(111), plt.imshow(cv2.cvtColor(debug, cv2.COLOR_BGR2RGB)), plt.title('max_contour')
        plt.show()
    height, width = img.shape
    pathfinding_matrix = []
    for h in range(height):
        row = []
        for w in range(width):
            if img[h, w] < 10:
                row.append(0)
            else:
                row.append(1)
        pathfinding_matrix.append(row)
    print(pathfinding_matrix)
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    holes = []
    for c in contours:
        if cv2.contourArea(c) != cv2.contourArea(max_contour) and cv2.contourArea(c) >= MIN_HOLE_AREA:
            holes.append(c)
            if DEBUG_HOLES:
                debug = np.zeros_like(img)
                cv2.drawContours(debug, [c], -1, (255,255,255), cv2.FILLED)
                _ = plt.subplot(111), plt.imshow(cv2.cvtColor(debug, cv2.COLOR_BGR2RGB)), plt.title('hole')
                plt.show()
    poly_no_holes = shapely.Polygon([[p[0][0], p[0][1]] for p in max_contour])
    poly_no_holes = poly_no_holes.buffer(0)
    if poly_no_holes.geom_type == 'MultiPolygon': # se poly non è un poligono ben definito provo a renderlo tale
        poly = max(poly_no_holes.geoms, key=lambda a: a.area)  
    holes_fixed = []
    for h in holes:
        hole = shapely.Polygon([[p[0][0], p[0][1]] for p in h])
        if hole.geom_type == 'MultiPolygon': # se poly non è un poligono ben definito provo a renderlo tale
            hole = max(hole.geoms, key=lambda a: a.area) 
        if poly_no_holes.contains(hole):
            holes_fixed.append(hole.buffer(0).exterior)
    poly = shapely.Polygon([[p[0][0], p[0][1]] for p in max_contour], holes=holes_fixed)
    poly = poly.buffer(0)
    if poly.geom_type == 'MultiPolygon': # se poly non è un poligono ben definito provo a renderlo tale
        poly = max(poly.geoms, key=lambda a: a.area)  
    if shapely.validation.explain_validity(poly) == "Valid Geometry":
        print_dat(poly, pathfinding_matrix)
    else:
        print(shapely.validation.explain_validity(poly))


if __name__ == '__main__':
    main()