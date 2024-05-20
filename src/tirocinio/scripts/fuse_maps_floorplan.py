import cv2
import argparse
import os
import matplotlib.pyplot as plt
import numpy as np

def update_image(image, fused_image):
    if fused_image is None:
        return image
    
    return cv2.addWeighted(image, 0.7, fused_image, 0.3, 0)

def check_positive(value):
    try:
        value = int(value)
        if value <= 0:
            raise argparse.ArgumentTypeError("{} is not a positive integer".format(value))
    except ValueError:
        raise Exception("{} is not an integer".format(value))
    return value

def parse_args():
    parser = argparse.ArgumentParser(description='Merge maps automatically.')
    parser.add_argument("-d", '--dir', default=os.getcwd(),
        help="Base directory to read maps from.")
    parser.add_argument("--gt-floorplan", default="",
        help="Ground truth floorplan to compare to.")
    parser.add_argument("--sample-test", default=False, action='store_true',
        help="Use this to fuse all the maps in the given directory. If false, it will try different sample sizes.")
    parser.add_argument("--step", type=check_positive, default=10, metavar="STEP", 
        help="Use this to increment sample size by STEP.")    
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

def fuse(samples: int, base_dir: str = os.getcwd(), gt_floorplan_path: str ="", complete: bool = False):

    gt_floorplan = None
    if gt_floorplan_path != "":
        gt_floorplan = cv2.imread(gt_floorplan_path, cv2.IMREAD_GRAYSCALE)

    floorplan = None
    addition = None

    sample_dir = ""
    if complete:
        print("Sample size complete.")
    else:
        sample_dir = f"{samples}_sample_size"
        print(f"Sample size equal to {samples}.")
        os.mkdir(sample_dir)

    image_paths = []
    for root, _, files in os.walk(base_dir):
        for f in files:
            if os.path.splitext(f)[1] in (".png", ".pgm") and len(image_paths) < samples:
                image_paths.append(os.path.join(root, f))
    
    max_height, max_width = max([cv2.imread(img_path, cv2.IMREAD_GRAYSCALE).shape[0] for img_path in image_paths]), max([cv2.imread(img_path, cv2.IMREAD_GRAYSCALE).shape[1] for img_path in image_paths])

    TOLERANCE = 0.07
    MAX_HEIGHT_ERROR = max_height * TOLERANCE
    MAX_WIDTH_ERROR = max_width * TOLERANCE

    to_initialize = True
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
        cv2.imwrite(os.path.join(os.getcwd(), sample_dir, "usate", os.path.basename(image_path)), image)
        floorplan = update_floorplan(floorplan, image.astype(np.uint8))

    print(f"Floorplan creation: used {used} out of {len(image_paths)} images.")

    if used == 0 or floorplan is None:
        print(f"FATAL: No image with acceptable error.")
        return

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
        #fused_image = update_image(image, fused_image)
        costs = cost_update(floorplan, image, costs)

    #floorplan_with_cost = np.array([[round((1-costs[h, w]) * 255) for w in range(width)] for h in range(height)])
    floorplan_with_cost = floorplan.copy()
    for h in range(max_height):
        for w in range(max_width):
            floorplan_with_cost[h, w] = round((1-costs[h, w]) * 255)

    addition = probabilize(addition, used).astype(np.uint8)
    cv2.imwrite(os.path.join(os.getcwd(), sample_dir, "addition.png"), addition)
    _, addition_thresholded = cv2.threshold(addition, 190, 255, cv2.THRESH_BINARY)

    blur = cv2.GaussianBlur(addition,(5,5),0)

    addition = cv2.bitwise_and(addition, floorplan)
    addition_thresholded = cv2.bitwise_and(addition_thresholded, floorplan)
    addition_thresholded = fill_holes(addition_thresholded)

    # Otsu's thresholding
    _, otsu = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    otsu = cv2.bitwise_and(otsu, floorplan)
    otsu = fill_holes(otsu)
    #print( "Otsu: {} {}".format(thresh,ret) )
    # Triangle algorithm thresholding
    _, tri = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_TRIANGLE)
    tri = cv2.bitwise_and(tri, floorplan)
    tri = fill_holes(tri)
    #print( "Triangle: {}".format(ret) )

    with open(os.path.join(os.getcwd(), sample_dir, "costs.txt"), "w") as costs_file:
        for h in range(max_height):
            for w in range(max_width):
                costs_file.write(f"{w} {h} {costs[h, w]}\n")

    cv2.imwrite(os.path.join(os.getcwd(), sample_dir, "threshold.png"), addition_thresholded)
    cv2.imwrite(os.path.join(os.getcwd(), sample_dir, "otsu.png"), otsu)
    cv2.imwrite(os.path.join(os.getcwd(), sample_dir, "tri.png"), tri)
    cv2.imwrite(os.path.join(os.getcwd(), sample_dir, "floorplan.png"), floorplan)
    cv2.imwrite(os.path.join(os.getcwd(), sample_dir, "added_map.png"), addition)

    col_1 = np.vstack([floorplan, addition_thresholded])
    col_2 = np.vstack([floorplan_with_cost, otsu])
    col_3 = np.vstack([addition, tri])
    summary = np.hstack([col_1, col_2, col_3])
    cv2.imwrite(os.path.join(os.getcwd(), sample_dir, "summary.png"), summary)

    if complete:
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

def main():

    args = parse_args()
    base_dir = args.dir
    gt_floorplan_path = args.gt_floorplan 
    sample_test = args.sample_test
    step = args.step


    max_size = 0
    for root, _, files in os.walk(base_dir):
        for f in files:
            if os.path.splitext(f)[1] in (".png", ".pgm"):
                max_size += 1

    if not sample_test:
        fuse(base_dir=base_dir, gt_floorplan_path=gt_floorplan_path, samples=max_size, complete=True)
    else: # naive
        for sample_size in range(step, max_size+1, step):
            fuse(base_dir=base_dir, gt_floorplan_path=gt_floorplan_path, samples=sample_size)
        if (max_size + 1) % step != 0:
            fuse(base_dir=base_dir, gt_floorplan_path=gt_floorplan_path, samples=max_size)

    
if __name__ == "__main__":
    main()