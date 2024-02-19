import cv2
import shapely
import shapely.validation
import matplotlib.pyplot as plt
import concurrent.futures
#import pyomo.environ as pyo
import numpy as np
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
    i, g1, guards = distanze_gg_data
    for i2, g2 in enumerate(guards):
        dis_gg.insert(i2, math.dist(g1,g2))
    return (i, dis_gg)

def min_distance_to_holes(poly_with_holes, point):
    return min([hole.distance(point) for hole in poly_with_holes.interiors])

def print_dat(poly):
    GUARD_RESOLUTION = 5
    WITNESS_NUMBER = 300
    MIN_DISTANCE_TO_POLY = 5
    MIN_COVERAGE = 0.80
    print("\ndata;")

    print_simple_param("coeff_coverage", 1)
    print_simple_param("coeff_distanze", 1)

    witnesses = [poly.exterior.line_interpolate_point(d, normalized=True) for d in np.linspace(0, 1, WITNESS_NUMBER)]
    nW = len(witnesses)
    guards = []
    # bounds Returns minimum bounding region (minx, miny, maxx, maxy)
    for x in range(round(poly.bounds[0]), round(poly.bounds[2]), GUARD_RESOLUTION):
        for y in range(round(poly.bounds[1]), round(poly.bounds[3]), GUARD_RESOLUTION):
            guard_candidate = shapely.Point([x, y])
            if poly.contains(guard_candidate) and poly.exterior.distance(guard_candidate) >= MIN_DISTANCE_TO_POLY and min_distance_to_holes(poly, guard_candidate) >= MIN_DISTANCE_TO_POLY:
                guards.append((x,y))
    nG = len(guards)
    print_simple_param("nW", nW)
    print_simple_param("nG", nG)
    
    copertura = []
    copribili = 0
    with concurrent.futures.ProcessPoolExecutor(max_workers=20) as executor:
        copertura_w_data = [(i, poly, w, guards) for i, w in enumerate(witnesses)]
        for i, cop_w in executor.map(copertura_w, copertura_w_data):
            copertura.insert(i, cop_w)
            if any(cop_w):
                copribili += 1
    
    min_coverage = min(MIN_COVERAGE, (copribili/nW)-0.1)
    print_simple_param("min_coverage", min_coverage)

    print_vector_param("costi_guardie", [(i+1, 1) for i in range(nG)])

    print_bidimensional_param("copertura", nW, nG, copertura)
    
    distanze = []
    with concurrent.futures.ProcessPoolExecutor(max_workers=20) as executor:
        distanze_gg_data = [(i, g1, guards) for i, g1 in enumerate(guards)]
        for i, dis_gg in executor.map(distanza_gg, distanze_gg_data):
            distanze.insert(i, dis_gg)
    print_bidimensional_param("distanza", nG, nG, distanze)

    print("\nend;")

    # Save guard location outside problem data since it's of no use (distances are precalculated)
    print_vector_param("posizione_guardie", [(i+1, (f"{x} {y}")) for (i, (x,y)) in enumerate(guards)])
    print_vector_param("posizione_testimoni", [(i+1, (f"{p.x} {p.y}")) for (i, p) in enumerate(witnesses)])

'''
def solve(poly):
    model = pyo.ConcreteModel()
    model.nW = pyo.Param(within=pyo.NonNegativeIntegers)
    model.nG = pyo.Param(within=pyo.NonNegativeIntegers)
    model.Witnesses = pyo.RangeSet(1, model.nW.values)
    model.Guards = pyo.RangeSet(1, model.nG)
    model.coeff_coverage = pyo.Param()
    model.coeff_distanze = pyo.Param()
    model.coeff_costo_guardie = pyo.Param()
    model.min_coverage = pyo.Param(within=pyo.PercentFraction)
    model.Costi_Guardie = pyo.RangeSet(1, model.nG, initialize=)
    g =
    model.Distanze = pyo.Set(initialize=model.Guards[model.Guards])
'''

def main():

    img_path = "/home/d-ber/catkin_ws/src/tirocinio/scripts/maps_agp/map_grey_to_black.png"
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
        _ = plt.subplot(111), plt.imshow(cv2.cvtColor(debug, cv2.COLOR_BGR2RGB)), plt.title('guards')
        plt.show()
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
        print_dat(poly)
    else:
        print(shapely.validation.explain_validity(poly))



if __name__ == '__main__':
    main()