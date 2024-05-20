from math import floor
import cv2
import argparse
import os
import matplotlib.pyplot as plt
import numpy as np

def update_image(image, fused_image):
    if fused_image is None:
        return image
    
    return cv2.addWeighted(image, 0.7, fused_image, 0.3, 0)

def parse_args():
    parser = argparse.ArgumentParser(description='Merge maps automatically.')
    parser.add_argument("-d", '--dir', default=os.getcwd(),
        help="Base directory to read maps from.")
    parser.add_argument("--gt-floorplan", default="",
        help="Ground truth floorplan to compare to.")
    return parser.parse_args()

def addimage(image, addition):
    if addition is None:
        return image

    return  image + addition

def probabilize(image, num):
    return image / num

def cost_update(floorplan, image, costs):
    HISTORICAL_WEIGHT = 0.8
    NEW_WEIGHT = 1 - HISTORICAL_WEIGHT
    height, width = floorplan.shape
    for h in range(height):
        for w in range(width):
            if image[h, w] in [0, 254, 255] and floorplan[h, w] != 0: # black or white on map and not black on floorplan    
                costs[h, w] = costs[h, w] * HISTORICAL_WEIGHT + (1 - (image[h, w]/255)) * NEW_WEIGHT
    return costs

def cost_initialize(floorplan):
    costs = np.zeros_like(floorplan, dtype=float)
    height, width = floorplan.shape
    for h in range(height):
        for w in range(width):
            if floorplan[h, w] == 0: # black
                costs[h, w] = 1
            elif floorplan[h, w] in {255,254} : # white
                costs[h, w] = 0
            else:
                print(f"ERROR, unexpected value in floorplan: {floorplan[h, w]}")
                exit()
    #with np.printoptions(threshold=np.inf):
    #    print(costs)

    return costs

def fill_holes(image):
    contours, _ = cv2.findContours(image, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        max_contour = max(contours, key = lambda cnt: cv2.contourArea(cnt))
        for c in contours:
            if cv2.contourArea(c) != cv2.contourArea(max_contour):
                image = cv2.drawContours(image, [c], -1, (0), thickness=-1)
        #_ = plt.subplot(111), plt.imshow(cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)), plt.title('filled')
        #plt.show()
        #exit()        
    return image

def init_floorplan(image, max_height, max_width):
    floorplan = np.zeros((max_height, max_width), dtype=np.uint8)
    points = (max_width, max_height)
    image = cv2.resize(image, points, interpolation= cv2.INTER_NEAREST)
    non_bw_mask = cv2.inRange(image, 1, 253)
    image[np.where(non_bw_mask == 255)] = [0]
    return cv2.bitwise_or(floorplan, image)

def update_floorplan(floorplan, image):
    max_height, max_width = floorplan.shape
    points = (max_width, max_height)

    image = cv2.resize(image, points, interpolation= cv2.INTER_NEAREST)
    non_bw_mask = cv2.inRange(image, 1, 253)
    image[np.where(non_bw_mask == 255)] = [0]
    return cv2.bitwise_or(floorplan, image)

def main():

    args = parse_args()
    base_dir = args.dir
    gt_floorplan_path = args.gt_floorplan 

    gt_floorplan = None
    if gt_floorplan_path != "":
        gt_floorplan = cv2.imread(gt_floorplan_path, cv2.IMREAD_GRAYSCALE)

    floorplan = None

    #fused_image = None
    addition = None
    m = 0

    image_paths = []
    for root, _, files in os.walk(base_dir):
        for f in files:
            if os.path.splitext(f)[1] in (".png", ".pgm"):
                image_paths.append(os.path.join(root, f))
    
    max_height, max_width = max([cv2.imread(img_path, cv2.IMREAD_GRAYSCALE).shape[0] for img_path in image_paths]), max([cv2.imread(img_path, cv2.IMREAD_GRAYSCALE).shape[1] for img_path in image_paths])

    TOLERANCE = 0.07
    MAX_HEIGHT_ERROR = max_height * TOLERANCE
    MAX_WIDTH_ERROR = max_width * TOLERANCE

    to_initialize = True
    #_ = plt.subplot(111), plt.imshow(cv2.cvtColor(floorplan, cv2.COLOR_GRAY2RGB)), plt.title(f"floorplan init")
    #plt.show()
    used = 0
    to_initialize = True
    for image_path in image_paths:
        image =  cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if abs(image.shape[0]-max_height) >= MAX_HEIGHT_ERROR or abs(image.shape[1]-max_width) >= MAX_WIDTH_ERROR:
            print(f"Floorplan creation: skipping image {os.path.basename(image_path)} with dim error {image.shape[0]-max_height}, {image.shape[1]-max_width}")
            continue
        elif to_initialize:
            floorplan = init_floorplan(image.astype(np.uint8), max_height, max_width)
            to_initialize = False
        used += 1
        cv2.imwrite(os.path.join("usate", os.path.basename(image_path)), image)
        floorplan = update_floorplan(floorplan, image.astype(np.uint8))

    print(f"Floorplan creation: used {used} out of {len(image_paths)} images.")

    #height, width = floorplan.shape
    points = (max_width, max_height)

    costs = cost_initialize(floorplan)

    for image_path in image_paths:
        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE) # 255=white, 205=gray, 0=black
        if abs(image.shape[0]-floorplan.shape[0]) >= MAX_HEIGHT_ERROR or abs(image.shape[1]-floorplan.shape[1]) >= MAX_WIDTH_ERROR:
            print(f"Image Merge: skipping image {os.path.basename(image_path)} with dim error h:{image.shape[0]-floorplan.shape[0]}, w:{image.shape[1]-floorplan.shape[1]}")
            continue
        else:
            print(f"Image Merge: acceptable error {os.path.basename(image_path)} with dim error h:{image.shape[0]-floorplan.shape[0]}, w:{image.shape[1]-floorplan.shape[1]}")
        image = cv2.resize(image, points, interpolation= cv2.INTER_NEAREST)

        # Fill the holes with black, only if closed (closed black areas)
        image = cv2.resize(image, points, interpolation= cv2.INTER_NEAREST)
        image = fill_holes(image)

        #_ = plt.subplot(121), plt.imshow(cv2.cvtColor(floorplan, cv2.COLOR_GRAY2RGB)), plt.title('floorplan')
        #_ = plt.subplot(122), plt.imshow(cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)), plt.title(f"image {os.path.basename(image_path)}")
        #plt.show()

        addition = addimage(image.astype(np.int32), addition)
        #print(f"Reading image {os.path.join(root, f)}")
        m += 1
        #fused_image = update_image(image, fused_image)
        costs = cost_update(floorplan, image, costs)

    #floorplan_with_cost = np.array([[round((1-costs[h, w]) * 255) for w in range(width)] for h in range(height)])
    floorplan_with_cost = floorplan.copy()
    for h in range(max_height):
        for w in range(max_width):
            floorplan_with_cost[h, w] = round((1-costs[h, w]) * 255)

    addition = probabilize(addition, m).astype(np.uint8)
    cv2.imwrite("addition.png", addition)
    _, addition_thresholded = cv2.threshold(addition, 190, 255, cv2.THRESH_BINARY)

    ## Otsu's Binarization thresholding as by https://docs.opencv.org/3.4/d7/d4d/tutorial_py_thresholding.html

    blur = cv2.GaussianBlur(addition,(5,5),0)

    addition = cv2.bitwise_and(addition, floorplan)
    addition_thresholded = cv2.bitwise_and(addition_thresholded, floorplan)
    addition_thresholded = fill_holes(addition_thresholded)

    # find normalized_histogram, and its cumulative distribution function
    hist = cv2.calcHist([blur],[0],None,[256],[0,256])
    hist_norm = hist.ravel()/hist.sum()
    Q = hist_norm.cumsum()
    bins = np.arange(256)
    fn_min = np.inf
    thresh = -1
    for i in range(1,256):
        p1,p2 = np.hsplit(hist_norm,[i]) # probabilities
        q1,q2 = Q[i],Q[255]-Q[i] # cum sum of classes
        if q1 < 1.e-6 or q2 < 1.e-6:
            continue
        b1,b2 = np.hsplit(bins,[i]) # weights
        # finding means and variances
        m1,m2 = np.sum(p1*b1)/q1, np.sum(p2*b2)/q2
        v1,v2 = np.sum(((b1-m1)**2)*p1)/q1,np.sum(((b2-m2)**2)*p2)/q2
        # calculates the minimization function
        fn = v1*q1 + v2*q2
        if fn < fn_min:
            fn_min = fn
            thresh = i
    # find otsu's threshold value with OpenCV function
    ret, otsu = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    otsu = cv2.bitwise_and(otsu, floorplan)
    otsu = fill_holes(otsu)
    #print( "Otsu: {} {}".format(thresh,ret) )
    # Triangle algorithm thresholding
    ret, tri = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_TRIANGLE)
    tri = cv2.bitwise_and(tri, floorplan)
    tri = fill_holes(tri)
    #print( "Triangle: {}".format(ret) )

    with open("costs.txt", "w") as costs_file:
        for h in range(max_height):
            for w in range(max_width):
                costs_file.write(f"{w} {h} {costs[h, w]}\n")

    cv2.imwrite("threshold.png", addition_thresholded)
    cv2.imwrite("otsu.png", otsu)
    cv2.imwrite("tri.png", tri)
    cv2.imwrite("floorplan.png", floorplan)
    cv2.imwrite("added_map.png", addition)

    if gt_floorplan is not None:
        _ = plt.subplot(331), plt.imshow(cv2.cvtColor(gt_floorplan, cv2.COLOR_GRAY2RGB)), plt.title('GT Floorplan')
        _ = plt.subplot(332), plt.imshow(cv2.cvtColor(floorplan, cv2.COLOR_GRAY2RGB)), plt.title(f'Extracted Floorplan with {TOLERANCE} Tolerance')
        _ = plt.subplot(333), plt.imshow(cv2.cvtColor(floorplan_with_cost, cv2.COLOR_GRAY2RGB)), plt.title('Floorplan with cost')
        _ = plt.subplot(334), plt.imshow(cv2.cvtColor(addition, cv2.COLOR_BGR2RGB)), plt.title('Added Map')
        _ = plt.subplot(335), plt.imshow(cv2.cvtColor(addition_thresholded, cv2.COLOR_BGR2RGB)), plt.title('Added Map Thresholded')
        _ = plt.subplot(337), plt.imshow(cv2.cvtColor(otsu, cv2.COLOR_GRAY2RGB)), plt.title('Added Map Otsu\'s Binarization')
        _ = plt.subplot(338), plt.imshow(cv2.cvtColor(tri, cv2.COLOR_GRAY2RGB)), plt.title('Added Map Triangle algorithm')
        plt.show()
    else:
        _ = plt.subplot(231), plt.imshow(cv2.cvtColor(floorplan, cv2.COLOR_GRAY2RGB)), plt.title('Floorplan')
        _ = plt.subplot(232), plt.imshow(cv2.cvtColor(floorplan_with_cost, cv2.COLOR_GRAY2RGB)), plt.title('Floorplan with cost')
        #_ = plt.subplot(232), plt.imshow(cv2.cvtColor(fused_image, cv2.COLOR_BGR2RGB)), plt.title(f'Fused Map ({m} maps)')
        _ = plt.subplot(233), plt.imshow(cv2.cvtColor(addition, cv2.COLOR_BGR2RGB)), plt.title('Added Map')
        _ = plt.subplot(234), plt.imshow(cv2.cvtColor(addition_thresholded, cv2.COLOR_BGR2RGB)), plt.title('Added Map Thresholded')
        _ = plt.subplot(235), plt.imshow(cv2.cvtColor(otsu, cv2.COLOR_GRAY2RGB)), plt.title('Added Map Otsu\'s Binarization')
        _ = plt.subplot(236), plt.imshow(cv2.cvtColor(tri, cv2.COLOR_GRAY2RGB)), plt.title('Added Map Triangle algorithm')
        plt.show()
    
if __name__ == "__main__":
    main()