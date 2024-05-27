import concurrent.futures
import math
import shapely
import shapely.validation
import cv2
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.path import Path
from matplotlib.patches import PathPatch
from matplotlib.collections import PatchCollection


# Plots a Polygon to pyplot `ax` (thanks to https://stackoverflow.com/questions/55522395/how-do-i-plot-shapely-polygons-and-objects-using-matplotlib)
def plot_polygon(ax, poly, **kwargs):
    path = Path.make_compound_path(
        Path(np.asarray(poly.exterior.coords)[:, :2]),
        *[Path(np.asarray(ring.coords)[:, :2]) for ring in poly.interiors])

    patch = PathPatch(path, **kwargs)
    collection = PatchCollection([patch], **kwargs)
    
    ax.add_collection(collection, autolim=True)
    ax.autoscale_view()
    return collection

def main():
    img_path = "/home/d-ber/catkin_ws/src/exp_cov/scripts/maps_agp/map_grey_to_black.png"
    MIN_HOLE_AREA = 10
    DEBUG_HOLES = False
    img = cv2.imread(img_path)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    contours, _ = cv2.findContours(img, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    max_contour = max(contours, key = lambda cnt: cv2.contourArea(cnt))
    holes = []
    for c in contours:
        if cv2.contourArea(c) != cv2.contourArea(max_contour) and cv2.contourArea(c) >= MIN_HOLE_AREA:
            holes.append(c)
            if DEBUG_HOLES:
                debug = np.zeros_like(img)
                cv2.drawContours(debug, [c], -1, (255,255,255), cv2.FILLED)
                _ = plt.subplot(111), plt.imshow(cv2.cvtColor(debug, cv2.COLOR_BGR2RGB)), plt.title('hole')
                plt.show()
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)      
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
        print("OK")
        fig, ax = plt.subplots()
        plot_polygon(ax, poly, facecolor='lightblue', edgecolor='red')
        plt.show()
    else:
        print(shapely.validation.explain_validity(poly))

if __name__ == '__main__':
    main()