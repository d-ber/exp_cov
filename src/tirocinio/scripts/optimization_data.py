import cv2
import shapely
import shapely.validation
import matplotlib.pyplot as plt
import pyomo.environ as pyo


def print_simple_param(name, value):
    print(f"param {name} := {value};")

def print_vector_param(name, pairs):
    print(f"param {name} := ")
    for (index, value) in pairs:
        print(f"{index} {value}")
    print(";")

def print_bidimensional_param(name, rows, cols, vals):
    print(f"param {name} : ", end="")
    for col in range(1, cols):
        print(f" {col} ", end="")
    print(f" := ")
    for row in range(1, rows):
        print(f"{row}")
        for col in range(1, cols):
            print(f"{vals[row, col]}", end="")
        print("")
    print(";")

def print_dat(poly):
    print_simple_param("coeff_coverage", 1)
    print_simple_param("coeff_distanze", 1)
    print_simple_param("coeff_costo_guardie", 1)
    print_simple_param("min_coverage", 1)

    witnesses = poly.exterior.segmentize(3).coords
    nW = len(witnesses)
    guards = []
    # bounds Returns minimum bounding region (minx, miny, maxx, maxy)
    for x in range(round(poly.bounds[0]), round(poly.bounds[2])):
        for y in range(round(poly.bounds[1]), round(poly.bounds[3])):
            if poly.contains(shapely.Point([x, y])) and poly.exterior.distance(shapely.Point([x, y])) >= 1:
                guards.append((x,y))
    nG = len(guards)
    print_simple_param("nW", nW)
    print_simple_param("nG", nG)

    #print_bidimensional_param("Copertura", )
    #print_bidimensional_param("Distanze", )
    

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

    img_path = "/home/aislab/catkin_ws/src/tirocinio/scripts/maps_agp/gt_smoothed.png"
    img = cv2.imread(img_path)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    contours, hierarchy = cv2.findContours(img, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    hierarchy = hierarchy[0]
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    poly = shapely.Polygon([[p[0][0], p[0][1]] for p in contours[0]])
    if shapely.validation.explain_validity(poly) == "Valid Geometry":
        print_dat(poly)
    else:
        print(shapely.validation.explain_validity(poly))



if __name__ == '__main__':
    main()