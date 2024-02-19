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
    return parser.parse_args()

def addimage(image, addition):
    if addition is None:
        return image

    return  image + addition

def probabilize(image, num):
    return image / num

def cost_update(floorplan, image, costs):
    HISTORICAL_WEIGHT = 0.5
    NEW_WEIGHT = 1 - HISTORICAL_WEIGHT
    height, width = floorplan.shape
    for h in range(height):
        for w in range(width):
            if image[h, w] in [0, 254, 255] and floorplan[h, w] != 0: # black or white on map and not black on floorplan    
                costs[h, w] = costs[h, w] * HISTORICAL_WEIGHT + (1 - ((image[h, w]/255) * NEW_WEIGHT))
    return costs

def cost_initialize(floorplan):
    costs = np.zeros_like(floorplan, dtype=float)
    height, width = floorplan.shape
    for h in range(height):
        for w in range(width):
            if floorplan[h, w] == 0: # black
                costs[h, w] = 1
            elif floorplan[h, w] == 255: # white
                costs[h, w] = 0
            else:
                print(f"ERROR, unexpected value in floorplan: {floorplan[h, w]}")
                exit()
    #with np.printoptions(threshold=np.inf):
    #    print(costs)

    return costs


def main():

    args = parse_args()
    base_dir = args.dir

    floorplan_path = "/home/d-ber/catkin_ws/src/tirocinio/scripts/maps_floorplan/gt_floorplan.png"
    floorplan = cv2.imread(floorplan_path, cv2.IMREAD_GRAYSCALE)

    fused_image = None
    addition = None
    m = 0
    costs = cost_initialize(floorplan)

    for root, _, files in os.walk(base_dir):
        for f in files:
            if os.path.splitext(f)[1] in (".png", ".pgm") and f != os.path.basename(floorplan_path):
                image = cv2.imread(os.path.join(root, f), cv2.IMREAD_GRAYSCALE) # 255=white, 205=gray, 0=black
                addition = addimage(image.astype(int), addition)
                print(f"Reading image {os.path.join(root, f)}")
                m += 1
                fused_image = update_image(image, fused_image)
                costs = cost_update(floorplan, image, costs)

    height, width = floorplan.shape
    #floorplan_with_cost = np.array([[round((1-costs[h, w]) * 255) for w in range(width)] for h in range(height)])
    floorplan_with_cost = floorplan.copy()
    for h in range(height):
        for w in range(width):
            floorplan_with_cost[h, w] = round((1-costs[h, w]) * 255)

    addition = probabilize(addition, m).astype(np.uint8)
    _, addition_thresholded = cv2.threshold(addition, 190, 255, cv2.THRESH_BINARY)

    ## Otsu's Binarization thresholding as by https://docs.opencv.org/3.4/d7/d4d/tutorial_py_thresholding.html

    blur = cv2.GaussianBlur(addition,(5,5),0)
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
    print( "Otsu: {} {}".format(thresh,ret) )
    # Triangle algorithm thresholding
    ret, tri = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_TRIANGLE)
    print( "Triangle: {}".format(ret) )

    _ = plt.subplot(231), plt.imshow(cv2.cvtColor(floorplan_with_cost, cv2.COLOR_GRAY2RGB)), plt.title('Floorplan with cost')
    _ = plt.subplot(232), plt.imshow(cv2.cvtColor(fused_image, cv2.COLOR_BGR2RGB)), plt.title(f'Fused Map ({m} maps)')
    _ = plt.subplot(233), plt.imshow(cv2.cvtColor(addition, cv2.COLOR_BGR2RGB)), plt.title('Added Map')
    _ = plt.subplot(234), plt.imshow(cv2.cvtColor(addition_thresholded, cv2.COLOR_BGR2RGB)), plt.title('Added Map Thresholded')
    _ = plt.subplot(235), plt.imshow(cv2.cvtColor(otsu, cv2.COLOR_GRAY2RGB)), plt.title('Added Map Otsu\'s Binarization')
    _ = plt.subplot(236), plt.imshow(cv2.cvtColor(tri, cv2.COLOR_GRAY2RGB)), plt.title('Added Map Triangle algorithm')
    plt.show()

                
    

if __name__ == "__main__":
    main()