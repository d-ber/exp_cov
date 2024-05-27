import cv2
import numpy as np
import matplotlib.pyplot as plt
import scipy.stats as st
import os
import time
import json
import rospkg
import argparse

# obj_img is a b&w image, in which the object is black and the background white
# img is the b&w image in which we translate obj_img
def translate_obj(obj_img, movement_area, img, dist_tra, dist_rot, show_steps, disable_rotation):
    obj_img = cv2.bitwise_not(obj_img)
    movement_area = cv2.cvtColor(movement_area, cv2.COLOR_BGR2RGB)
    height, width = obj_img.shape[:2]

    to_translate = True

    translated_image = []
    dst = np.empty(1)
    dx = 0
    dy = 0
    angle = 0
    tries = 0
    MAX_ATTEMPTS = 150

    while to_translate:

        # Generate random values with given distributions
        dx = dist_tra.rvs()
        dy = dist_tra.rvs()
        if not disable_rotation:
            angle = dist_rot.rvs()

        # Create the translation matrix using dx and dy, it is a Numpy array 
        translation_matrix = np.array([
            [1, 0, dx],
            [0, 1, dy]
        ], dtype=np.float32)
        translated_image = cv2.warpAffine(src=obj_img, M=translation_matrix, dsize=(width, height))
        translated_image = cv2.cvtColor(translated_image, cv2.COLOR_BGR2RGB)

        if not disable_rotation:
            # Create the rotational matrix
            center = (translated_image.shape[1]//2, translated_image.shape[0]//2)
            scale = 1
            rot_mat = cv2.getRotationMatrix2D(center, angle, scale)
            translated_image = cv2.warpAffine(src=translated_image, M=rot_mat, dsize=(width, height))

        translated_image = cv2.threshold(translated_image, 127, 255, cv2.THRESH_BINARY)[1]
        translated_image = cv2.bitwise_not(translated_image)

        black_pixels_obj = np.count_nonzero(cv2.bitwise_not(translated_image))
        # Check if object is outside the map
        if black_pixels_obj == 0:
            continue
        black_pixels_overlapped = np.count_nonzero(cv2.bitwise_and(cv2.bitwise_not(translated_image), cv2.bitwise_not(img)))
        overlap_percentage = 100*(black_pixels_overlapped/black_pixels_obj)

        movement_area_overlapped = np.count_nonzero(cv2.bitwise_and(cv2.bitwise_not(translated_image), cv2.bitwise_not(movement_area)))
        movement_area_overlap_percentage = 100*(movement_area_overlapped/black_pixels_obj)

        dst = cv2.bitwise_and(translated_image, img)

        # If at least 80% of object pixels don't overlap and the object is at least 95% inside the defined movement area, we accept the translation
        if overlap_percentage < 20 and movement_area_overlap_percentage >= 95:
            to_translate = False
        # Elif too many translations have been tried unsuccessfully, keep obj in original position
        elif tries > MAX_ATTEMPTS:
            dst = cv2.bitwise_and(cv2.bitwise_not(cv2.cvtColor(obj_img, cv2.COLOR_BGR2RGB)), img)
            to_translate = False
        tries += 1

    if show_steps:
        _ = plt.subplot(231), plt.imshow(cv2.bitwise_not(obj_img), cmap='gray'), plt.title('Original Object')
        _ = plt.subplot(232), plt.imshow(movement_area,cmap='gray'), plt.title('Movement Area')
        _ = plt.subplot(233), plt.imshow(translated_image, cmap='gray'), plt.title('Translated Object')
        _ = plt.subplot(234), plt.imshow(img, cmap='gray'), plt.title('Original Image')
        _ = plt.subplot(235), plt.imshow(dst, cmap='gray'), plt.title('Merged Image')
        plt.show()

    return (dst, dx, dy)


def extract_color_pixels(image, movement_mask_image, rectangles_path, show_recap=False, show_steps=False, save_map=False, sizex=20, sizey=20, silent=False):
    image_objects_removed = image.copy()
    
    # Convert RGB image to HSV (Hue, Saturation, Value) color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hsv_movement = cv2.cvtColor(movement_mask_image, cv2.COLOR_BGR2HSV)

    # Define the HSV range based on the specified color
    # (red, green, blue) that is (oggetti semistatici, aree di disturbo, clutter)
    RED = "red"
    GREEN = "green"
    BLUE = "blue"
    colors = (RED, GREEN, BLUE)
    lower_ranges = (np.array([0, 100, 100]), np.array([40, 40, 40]), np.array([100, 50, 50]))
    upper_ranges = (np.array([10, 255, 255]), np.array([80, 255, 255]), np.array([140, 255, 255]))

    #movement_color = ("yellow")
    lower_ranges_movement = np.array([20, 100, 100])
    upper_ranges_movement = np.array([40, 255, 255])
    # Create a binary mask for the specified color for movement areas
    color_mask_movement = cv2.inRange(hsv_movement, lower_ranges_movement, upper_ranges_movement) 
    # Find contours in the mask
    contours_movement = cv2.findContours(color_mask_movement, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]

    # HSV ranges for closed doors (orange) and open doors (purple)
    ORANGE = "orange"
    PURPLE = "purple"
    door_colors = (ORANGE, PURPLE) 
    lower_door_range = (np.array([10, 100, 100]), np.array([130, 50, 50]))
    upper_door_range = (np.array([20, 255, 255]), np.array([160, 255, 255]))

    # To work with doors
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    closed_doors_mask = cv2.inRange(hsv, lower_door_range[door_colors.index(ORANGE)], upper_door_range[door_colors.index(ORANGE)])
    closed_doors_mask_dilated = cv2.dilate(closed_doors_mask, kernel, iterations=1)
    open_doors_mask = cv2.inRange(hsv, lower_door_range[door_colors.index(PURPLE)], upper_door_range[door_colors.index(PURPLE)])
    open_doors_mask_dilated = cv2.dilate(open_doors_mask, kernel, iterations=1)

    open_doors = cv2.findContours(open_doors_mask_dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    closed_doors = cv2.findContours(closed_doors_mask_dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]

    # Probability for orange/purple doors (0.05 prob to have a close door -> 0.95 of having it open)
    DOOR_PROB = 0.05
    bernoulli_doors = st.bernoulli(DOOR_PROB)

    hsv = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    image_objects_removed[np.where(closed_doors_mask > np.array(0))] = [255, 255, 255]
    hsv[np.where(closed_doors_mask > np.array(0))] = [255, 255, 255]
    image_objects_removed[np.where(open_doors_mask > np.array(0))] = [255, 255, 255]
    hsv[np.where(open_doors_mask > np.array(0))] = [255, 255, 255]
    hsv = cv2.cvtColor(hsv, cv2.COLOR_BGR2HSV)

    color_masks = []
    images_with_boxes = []
    results = []
    objs = []
    contours = []
    contour_obj_image_movement_area = []
    j = 0
    all_image_mask = np.zeros_like(image)

    # For each color, we extract objs and associate each with the correct movement area
    for i in range (0, 3):
        # Create a binary mask for the specified color
        color_masks.insert(i, cv2.inRange(hsv, lower_ranges[i], upper_ranges[i]))

        # Find contours in the mask
        contours.insert(i, cv2.findContours(color_masks[i], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0])
        objs.insert(i, len(contours[i]))

        images_with_boxes.insert(i, cv2.cvtColor(color_masks[i], cv2.COLOR_GRAY2BGR))

        # Draw bounding boxes around each object
        for contour in contours[i]:
            # Get the minimum area rectangle that bounds the contour
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.asarray(box, dtype=np.intp)

            # Draw the bounding box
            cv2.drawContours(images_with_boxes[i], [box], 0, (0, 255, 0), 2)  # Draw a green rectangle

            found = False

            # We look for the right association between objs and movement area
            for contour_movement in contours_movement:
                # Draw object on blank canvas
                mask = np.zeros_like(color_masks[i])
                cv2.drawContours(mask, [contour], -1, (255,255,255), cv2.FILLED)
                # Extract the object using the mask
                object_image = cv2.bitwise_and(color_masks[i], color_masks[i], mask=mask)
                object_image = cv2.bitwise_not(object_image)
                black_pixels_obj = np.count_nonzero(cv2.bitwise_not(object_image))

                # Draw areas on blank canvas
                mask = np.zeros_like(color_mask_movement)
                cv2.drawContours(mask, [contour_movement], -1, (255,255,255), cv2.FILLED)
                # Extract the object using the mask
                movement_area = cv2.bitwise_and(color_mask_movement, color_mask_movement, mask=mask)
                movement_area = cv2.bitwise_not(movement_area)
                black_pixels_overlapped = np.count_nonzero(cv2.bitwise_and(cv2.bitwise_not(object_image), cv2.bitwise_not(movement_area)))
                overlap_percentage = 100*(black_pixels_overlapped/black_pixels_obj)
                
                if overlap_percentage == 100:
                    contour_obj_image_movement_area.insert(j, (contour, i, object_image, movement_area))
                    j = j+1
                    found = True
                    break
            
            if not found:
                # Draw object on blank canvas
                mask = np.zeros_like(color_masks[i])
                cv2.drawContours(mask, [contour], -1, (255,255,255), cv2.FILLED)
                # Extract the object using the mask
                object_image = cv2.bitwise_and(color_masks[i], color_masks[i], mask=mask)
                object_image = cv2.bitwise_not(object_image)

                # If object is in no movement area, give movement area equal to full image
                contour_obj_image_movement_area.insert(j, (contour, i, object_image, all_image_mask))

        # Apply the mask to the original image
        results.insert(i, cv2.bitwise_and(image_objects_removed, image_objects_removed, mask=color_masks[i]))

        # Set the pixels in the original image where the color is extracted to white
        image_objects_removed[np.where(color_masks[i] > 0)] = [255, 255, 255]

    translated_objs_image = image_objects_removed
    
    # For each door, we either keep it closed or open, by associating closed and open configs for each door, assuming they as adjacent
    for closed_door in closed_doors:
        for open_door in open_doors:
            mask_open = np.zeros_like(hsv)
            cv2.drawContours(mask_open, [open_door], -1, (255,255,255), cv2.FILLED)
            mask_closed = np.zeros_like(hsv)
            cv2.drawContours(mask_closed, [closed_door], -1, (255,255,255), cv2.FILLED)
            overlapped = cv2.bitwise_and(mask_open, mask_closed)
            overlap = np.count_nonzero(overlapped)
            if overlap > 0:
                mask_eroded = cv2.erode(mask_closed, kernel, iterations=1) if bernoulli_doors.rvs() else cv2.erode(mask_open, kernel, iterations=1)
                translated_objs_image = cv2.bitwise_and(translated_objs_image, cv2.bitwise_not(mask_eroded))
                break

    # Translational probability red and blue obstacles
    MEAN_TRA = 0
    STD_TRA = 10
    norm_tra = st.norm(loc=MEAN_TRA, scale=STD_TRA)

    # Translational probability green areas
    MEAN_GREEN = 0
    STD_GREEN = 0.1
    norm_green = st.norm(loc=MEAN_GREEN, scale=STD_GREEN)

    # Rotational probability
    MEAN_ROT = 0
    STD_ROT = 20
    norm_rot = st.norm(loc=MEAN_ROT, scale=STD_ROT)

    # Probability for blue objects appearance
    CLUTTER_PROB = 0.5
    bernoulli_clutter = st.bernoulli(CLUTTER_PROB)

    rectangles_info = []
    contours_green_translated = list(contours[colors.index(GREEN)])
    green_idx = 0
    image_width = image.shape[1]
    image_height = image.shape[0]
    size_width = sizey
    size_height = sizex

    for j, (contour, obj_color_idx, object_image, movement_area) in enumerate(contour_obj_image_movement_area):
        if colors[obj_color_idx] == GREEN:
            # Get dx and dy translation so that at least 80% does not overlap
            _, dx, dy = translate_obj(object_image, movement_area, translated_objs_image, dist_tra=norm_green, dist_rot=norm_rot, show_steps=show_steps, disable_rotation=True)

            for r in range(4):
                contours_green_translated[green_idx][r][0][0] += dx
                contours_green_translated[green_idx][r][0][1] += dy

            # Calculate the bounding rectangle
            x, y, w, h = cv2.boundingRect(contour)
            
            # Translate it 
            x = x + dx 
            y = y + dy

            # Get center coordinates
            center_x = x + (w / 2)
            center_y = y + (h / 2)

            # Convert to stage coordinates
            center_x = (-size_width/2) + ((size_width/2 - (-size_width/2)) / (image_width - 0)) * (center_x - 0)
            center_y = (-size_height/2) + ((size_height/2 - (-size_height/2)) / (image_height - 0)) * ((image_height-center_y) - 0)

            # Convert pixel units to the desired unit (n pixels per unit)
            w = size_width*(w/image_width)
            h = size_height*(h/image_height)

            # Add rectangle information to the list
            rectangles_info.append({
                "center": {
                    "x": center_x,
                    "y": center_y,
                    "z": 0},
                "width": w,
                "height": h
            })
            green_idx += 1
        elif colors[obj_color_idx] == BLUE and bernoulli_clutter.rvs():
            # If object is clutter and luck commands it, we skip it
            continue
        else: # Else colors[obj_color_idx] == RED
            translated_objs_image, _, _ = translate_obj(object_image, movement_area, translated_objs_image, dist_tra=norm_tra, dist_rot=norm_rot, show_steps=show_steps, disable_rotation=False)

    green_objs_translated = image_objects_removed.copy()
    green_objs_translated = cv2.drawContours(green_objs_translated, contours_green_translated, -1, (0, 255, 0), cv2.FILLED)

    # Display the original image and the result, along some informational images
    if show_recap:
        _ = plt.subplot(331), plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB)), plt.title('Original Image')
        _ = plt.subplot(334), plt.imshow(color_masks[0], cmap='gray'), plt.title(f"{colors[0].title()} Pixels Mask")
        _ = plt.subplot(335), plt.imshow(color_masks[1], cmap='gray'), plt.title(f"{colors[1].title()} Pixels Mask")
        _ = plt.subplot(336), plt.imshow(color_masks[2], cmap='gray'), plt.title(f"{colors[2].title()} Pixels Mask")
        _ = plt.subplot(337), plt.imshow(cv2.cvtColor(movement_mask_image, cv2.COLOR_BGR2RGB)), plt.title('Movement Mask')
        _ = plt.subplot(338), plt.imshow(cv2.cvtColor(color_mask_movement, cv2.COLOR_BGR2RGB)), plt.title('Movement Pixels Mask')
        _ = plt.subplot(339), plt.imshow(cv2.cvtColor(green_objs_translated, cv2.COLOR_BGR2RGB)), plt.title('Green areas translated')
        _ = plt.subplot(332), plt.imshow(cv2.cvtColor(image_objects_removed, cv2.COLOR_BGR2RGB)), plt.title('Without colored Pixels')
        _ = plt.subplot(333), plt.imshow(cv2.cvtColor(translated_objs_image, cv2.COLOR_BGR2RGB)), plt.title('Colored Objects translated')
        plt.show()

    if save_map:
        # Convert the list to JSON format
        json_data = json.dumps(rectangles_info, indent=4)

        # Write JSON data to a file
        with open(rectangles_path, 'w') as json_file:
            json_file.write(json_data)
            if not silent:
                print(f"Saved rectangles info as {rectangles_path}")

    return translated_objs_image

def check_positive(value):
    try:
        value = int(value)
        if value <= 0:
            raise argparse.ArgumentTypeError("{} is not a positive integer".format(value))
    except ValueError:
        raise Exception(f"{value} is not an integer")
    return value

def check_positive_float(value):
    try:
        value = float(value)
        if value <= 0:
            raise argparse.ArgumentTypeError("{} is not a positive float".format(value))
    except ValueError:
        raise Exception(f"{value} is not a float")
    return value

def check_positive_or_zero(value):
    try:
        value = int(value)
        if value < 0:
            raise argparse.ArgumentTypeError("{} is not a positive integer nor zero".format(value))
    except ValueError:
        raise Exception(f"{value} is not an integer")
    return value

def check_pose(value):
    try:
        ret_value = value.split()
        if len(ret_value) != 2:
            raise argparse.ArgumentTypeError(f"Given pose value \"{value}\" is not made of 2 numbers")
        return (float(ret_value[0]), float(ret_value[1]))
    except ValueError:
        raise Exception(f"{value} is not made of 2 numbers")


def parse_args():
    # Get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()
    # Get the file path for exp_cov
    package_path = rospack.get_path('exp_cov')

    parser = argparse.ArgumentParser(description='Modify rgb maps automatically.')
    parser.add_argument('--map', default=os.path.join(package_path, "maps_rgb_lab/map1/map1_rgb.png"),
        help="Path to the rgb map file.", metavar="MAP_PATH")
    parser.add_argument('--mask', default=os.path.join(package_path, "maps_rgb_lab/map1/map1_movement_mask.png"),
        help="Path to the rgb mask map file for movement areas. Each rgb object in the map will be moved within the yellow mask given in this file. If the object is none, then it can move freely.", metavar="MASK_PATH")
    parser.add_argument('--show', action='store_true',
        help="Use this to show a final recap.")
    parser.add_argument('--save', action='store_true',
        help="Use this to save the produced map and rectangles info.")
    parser.add_argument("-b", "--batch", type=check_positive, default=1, metavar="N",
        help="Use this to produce N maps and save them.")    
    parser.add_argument("--worlds", type=check_positive_or_zero, default=0, metavar="N",
        help="Use this to produce N maps and save them.")    
    parser.add_argument('--no-timestamp', action='store_true',
        help="""Use this save a single image without timestamp. If image.png already exists, it will create image_n.png
            in which n is the smallest number so that there is no file with that name. Same goes for the rectangles file.""")
    parser.add_argument("-d", '--dir', default=os.getcwd(),
        help="Base directory to save files in.")
    parser.add_argument('--steps', action='store_true',
        help="Use this to show the processing steps.")
    parser.add_argument("--speedup", type=check_positive, default=10, metavar="SPEEDUP",
        help="Use this to adjust stage simulation speed. Higher is faster but heavier on the CPU.")
    parser.add_argument('--pose', type=check_pose, default=(0, 0), metavar="X Y",
        help="Robot pose X and Y coordinates.")
    parser.add_argument('--scale', type=check_positive_float, default=0.035888, metavar="PIXELS",
        help="Number of meters per pixel in png map.")
    parser.add_argument("--world-num", type=check_positive_or_zero, default=None, metavar="N",
        help="Use this to produce a maps that ends in N and save it. Setting this argument overrides --worlds.")    
    parser.add_argument('--silent', action='store_true',
        help="Use this to avoid printing info.")   
    return parser.parse_args()

def get_world_text(image, name, speedup, pose, scale, sizex, sizey):
    return f"""
    # World {name}
    define turtlebot3 position
    (
        size [ 0.138 0.178 0.192 ] # size [ x:<float> y:<float> z:<float> ]

        # this block approximates shape of a Turtlebot3
        block( 
            points 6
            point[0] [ 0.6 0 ]
            point[1] [ 1 0 ]
            point[2] [ 1 1 ]
            point[3] [ 0.6 1 ]
            point[4] [ 0 0.7 ]
            point[5] [ 0 0.3 ]
        )
        color "black"
    )

    define create turtlebot3( color "gray90" )

    define sick ranger
    (
        sensor(
            # ranger-specific properties
            range [ 0.06 15 ]
            fov 359
            samples 1022

            # noise [range_const range_prop angular]
            # range_const - describes constant noise. This part does not depends on range
            # range_prop - proportional noise, it scales with reading values
            # angular - bearing error. For the cases when reading angle is not accurate

            noise [ 0.0 0.01 0.01 ]
        )

        # generic model properties
        color "blue"
        size [ 0.0695 0.0955 0.0395 ] # dimensions from LDS-01 data sheet	
    )


    define floorplan model
    (
        # sombre, sensible, artistic
        color "gray10"

        # most maps will need a bounding box
        boundary 1

        gui_nose 0
        gui_grid 0
        gui_move 0
        gui_outline 0
        gripper_return 0
        fiducial_return 0
        ranger_return 1
    )

    quit_time 360000 # 100 hours of simulated time
    speedup {speedup}

    paused 0

    resolution 0.01

    # configure the GUI window
    window
    (
        size [ 700.000 660.000 ] # in pixels (size [width height])
        scale {1/scale}  # pixels per meter
        center [ 0  0 ]
        rotate [ 0  0 ]
                    
        show_data 1              # 1=on 0=off
    )

    # load an environment bitmap
    floorplan
    ( 
        name  "turtlebot3-stage"
        size [ {sizey} {sizex} 1.0 ] # size [x y z]
        pose [0 0 0 0]
        bitmap "bitmaps/image{image}.png" # bitmap: solo il nero è renderizzato
    )
    turtlebot3
    (		  
        # can refer to the robot by this name
        name  "turtlebot3"
        pose [ {pose[0]} {pose[1]} 0 0 ] 

        sick()
    )
    """

def get_non_existent_filename(base_dir, no_timestamp):
    filename = "src/"
    if no_timestamp:
        filename = os.path.join(base_dir, "image.png")    
        i = 1
        while os.path.exists(filename): 
            i += 1
            filename = os.path.join(base_dir, f"image_{i}.png")
    else:
        filename = os.path.join(base_dir, f"image_{time.time_ns()}.png")
        while os.path.exists(filename): 
            filename = os.path.join(base_dir, f"image_{time.time_ns()}.png")
    return filename

def main():

    args = parse_args()
    image_path = args.map
    show_recap = args.show
    show_steps = args.steps
    save_map = args.save
    batch = args.batch
    no_timestamp = args.no_timestamp
    base_dir = args.dir
    movement_mask_image_path = args.mask
    worlds = args.worlds
    speedup = args.speedup
    pose = args.pose
    scale = args.scale
    world_num = args.world_num
    silent = args.silent


    movement_mask_image = cv2.imread(movement_mask_image_path, cv2.IMREAD_COLOR)
        
    image = cv2.imread(image_path, cv2.IMREAD_COLOR)

    sizex = image.shape[0]
    sizex = sizex/(1/scale)
    sizey = image.shape[1]
    sizey = sizey/(1/scale)

    if worlds == 0 and world_num is None:
        if batch == 1:
            rectangles_path = os.path.join(base_dir, "rectangles.json") if no_timestamp else os.path.join(base_dir, f"rectangles_{time.time_ns()}.json")
            image_modified = extract_color_pixels(image, movement_mask_image, rectangles_path, show_recap=show_recap, show_steps=show_steps, save_map=save_map, sizex=sizex, sizey=sizey, silent=silent)

            if save_map:
                filename = get_non_existent_filename(base_dir, no_timestamp)                
                cv2.imwrite(filename, image_modified)
                if not silent:
                    print(f"Saved map as {filename}")
        else:
            for i in range(batch):
                rectangles_path = os.path.join(base_dir, f"rectangles_{i}.json")
                image_modified = extract_color_pixels(image, movement_mask_image, rectangles_path, show_recap=show_recap, show_steps=show_steps, save_map=True, sizex=sizex, sizey=sizey, silent=silent)
                filename = os.path.join(base_dir, f"image_{i}.png")
                cv2.imwrite(filename, image_modified)
                if not silent:
                    print(f"Saved map as {filename}")
    else:
        world_range = None
        if world_num is not None:
            world_range = [world_num]
        else:
            world_range = range(0, worlds)
        name = os.path.basename(os.path.splitext(image_path)[0])
        for i in world_range:
            rectangles_path = os.path.join(base_dir, f"bitmaps/rectangles{i}.json")
            bitmaps_dir = os.path.join(base_dir, "bitmaps")
            if not os.path.exists(bitmaps_dir) or not os.path.isdir(bitmaps_dir):
                os.makedirs(bitmaps_dir)
            image_modified = extract_color_pixels(image, movement_mask_image, rectangles_path, show_recap=show_recap, show_steps=show_steps, save_map=True, sizex=sizex, sizey=sizey, silent=silent)
            filename = os.path.join(base_dir, f"bitmaps/image{i}.png")
            cv2.imwrite(filename, image_modified)
            if not silent:
                    print(f"Saved map as {filename}")
            worldfile_path = os.path.join(base_dir, f"world{i}.world")
            with open(worldfile_path, "w", encoding="utf-8") as worldfile:
                worldfile.write(get_world_text(i, name, speedup, pose, scale, sizex, sizey))
                if not silent:
                    print(f"Saved worldfile as {worldfile_path}")

if __name__ == "__main__":
    main()