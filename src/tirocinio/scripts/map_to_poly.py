import cv2
from shapely import equals_exact, geometry
from shapely import validation
import matplotlib.pyplot as plt

def shapely_poly_to_966_format(poly):
    coords = poly.exterior.coords
    # coords[:-1] per evitare di tornare al punto di partenza
    print(len(coords[:-1]), end=" ")
    for x, y in coords[:-1]:
        print(f"{int(x)}/1 {int(y)}/1", end=" ")

def print_coordinates(contour, end=" "):
    visti = set()
    for p in contour:
        if (p[0][0], p[0][1]) in visti:
            continue
            #print(f"AHH, ecco:{p[0][0]}/1 {p[0][1]}/1")
        else:
            visti.add((p[0][0], p[0][1]))
        print(f"{p[0][0]}/1 {p[0][1]}/1", end=end)

def main():

    img_path = "/home/d-ber/catkin_ws/src/tirocinio/scripts/maps_agp/gt_smoothed.png"
    img = cv2.imread(img_path)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    contours, hierarchy = cv2.findContours(img, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    hierarchy = hierarchy[0]
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    #print(len(contours))
    #print(len(hierarchy))
    #print(len(contours[0]), end=" ")
    #print_coordinates(contours[0], end=" ")
    poly = geometry.Polygon([[p[0][0], p[0][1]] for p in contours[0]])
    if validation.explain_validity(poly) == "Valid Geometry":
        #print(poly)
        shapely_poly_to_966_format(poly)
    else:
        print(validation.explain_validity(poly))
    #temp = img.copy()
    #cv2.drawContours(temp, [contours[0]], -1, (0, 255, 0), 1)
    #_ = plt.subplot(111), plt.imshow(cv2.cvtColor(temp, cv2.COLOR_BGR2RGB)), plt.title('contours')
    #plt.show()
    #print(len(contours[1:]), end=" ")
    #for i, c in enumerate(contours[1:]):
        #temp = img.copy()
        # Formato richiesto
        # numero_vertici_esterni x1/y1 x2/y2 ... xn/yn numero_buchi numero_vertici_buco1 x1/y1 ... xm/ym ... numero_vertici_bucoz x1/y1 ... xk/yk
        #print(len(c), end=" ")
        #print_coordinates(c, end=" ")
        #cv2.drawContours(temp, [c], -1, (0, 255, 0), 1)
        #print(i, hierarchy[i])
        #_ = plt.subplot(111), plt.imshow(cv2.cvtColor(temp, cv2.COLOR_BGR2RGB)), plt.title('contours')
        #plt.show()


if __name__ == '__main__':
    main()