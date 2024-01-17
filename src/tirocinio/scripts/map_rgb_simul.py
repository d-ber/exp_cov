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
# img is an image
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
    TRIES_LIMIT = 150

    while to_translate:

        # generate random values with given distributions
        dx = dist_tra.rvs()
        dy = dist_tra.rvs()
        if not disable_rotation:
            angle = dist_rot.rvs()

        # create the translation matrix using dx and dy, it is a NumPy array 
        translation_matrix = np.array([
            [1, 0, dx],
            [0, 1, dy]
        ], dtype=np.float32)
        translated_image = cv2.warpAffine(src=obj_img, M=translation_matrix, dsize=(width, height))
        translated_image = cv2.cvtColor(translated_image, cv2.COLOR_BGR2RGB)

        if not disable_rotation:
            # create the rotational matrix
            center = (translated_image.shape[1]//2, translated_image.shape[0]//2)
            scale = 1
            rot_mat = cv2.getRotationMatrix2D(center, angle, scale)
            translated_image = cv2.warpAffine(src=translated_image, M=rot_mat, dsize=(width, height))

        translated_image = cv2.threshold(translated_image, 127, 255, cv2.THRESH_BINARY)[1]
        translated_image = cv2.bitwise_not(translated_image)

        black_pixels_obj = np.count_nonzero(cv2.bitwise_not(translated_image))
        black_pixels_overlapped = np.count_nonzero(cv2.bitwise_and(cv2.bitwise_not(translated_image), cv2.bitwise_not(img)))
        overlap_percentage = 100*(black_pixels_overlapped/black_pixels_obj)
        plt.show()

        movement_area_overlapped = np.count_nonzero(cv2.bitwise_and(cv2.bitwise_not(translated_image), cv2.bitwise_not(movement_area)))
        movement_area_overlap_percentage = 100*(movement_area_overlapped/black_pixels_obj)

        dst = cv2.bitwise_and(translated_image, img)

        # If at least 80% of object pixels don't overlap and the object is at least 95% inside the defined movement area, we accept the translation
        if overlap_percentage < 20 and movement_area_overlap_percentage >= 95:
            to_translate = False
        # Elif keep obj in original position if too many translations have been tried unsuccessfully
        elif tries > TRIES_LIMIT:
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


def extract_color_pixels(image, movement_mask_image, rectangles_path, show_recap=False, show_steps=False, save_map=False):
    # Copy
    image_objects_removed = image.copy()
    
    # Convert RGB image to HSV (Hue, Saturation, Value) color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # Convert RGB image to HSV (Hue, Saturation, Value) color space
    hsv_movement = cv2.cvtColor(movement_mask_image, cv2.COLOR_BGR2HSV)

    # Define the HSV range based on the specified color
    # (red, green, blue) that is (oggetti semistatici, aree di disturbo, clutter)
    colors = ("red", "green", "blue")
    lower_ranges = (np.array([0, 100, 100]), np.array([40, 40, 40]), np.array([100, 50, 50]))
    upper_ranges = (np.array([10, 255, 255]), np.array([80, 255, 255]), np.array([140, 255, 255]))

    #movement_color = ("yellow")
    lower_ranges_movement = np.array([20, 100, 100])
    upper_ranges_movement = np.array([40, 255, 255])
    # Create a binary mask for the specified color for movement areas
    color_mask_movement = cv2.inRange(hsv_movement, lower_ranges_movement, upper_ranges_movement) 
    # Find contours in the mask
    contours_movement = cv2.findContours(color_mask_movement, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]

    color_masks = []
    images_with_boxes = []
    results = []
    objs = []
    contours = []
    contour_obj_image_movement_area = []
    j = 0
    all_image_mask = np.zeros_like(image)

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

            for contour_movement in contours_movement:
                # draw object on blank canvas
                mask = np.zeros_like(color_masks[i])
                cv2.drawContours(mask, [contour], -1, (255,255,255), cv2.FILLED)# Extract the object using the mask
                object_image = cv2.bitwise_and(color_masks[i], color_masks[i], mask=mask)
                object_image = cv2.bitwise_not(object_image)
                black_pixels_obj = np.count_nonzero(cv2.bitwise_not(object_image))

                # draw areas on blank canvas
                mask = np.zeros_like(color_mask_movement)
                cv2.drawContours(mask, [contour_movement], -1, (255,255,255), cv2.FILLED)# Extract the object using the mask
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
                # draw object on blank canvas
                mask = np.zeros_like(color_masks[i])
                cv2.drawContours(mask, [contour], -1, (255,255,255), cv2.FILLED)# Extract the object using the mask
                object_image = cv2.bitwise_and(color_masks[i], color_masks[i], mask=mask)
                object_image = cv2.bitwise_not(object_image)

                #If object is in no movement area, give movement area equal to full image
                contour_obj_image_movement_area.insert(j, (contour, i, object_image, all_image_mask))

            # Apply the mask to the original image
        results.insert(i, cv2.bitwise_and(image_objects_removed, image_objects_removed, mask=color_masks[i]))

        # Set the pixels in the original image where the color is extracted to white
        image_objects_removed[np.where(color_masks[i] > 0)] = [255, 255, 255]

    # translational probability red and blu obstacles
    mean_tra = 0
    std_tra = 10
    norm_tra = st.norm(loc=mean_tra, scale=std_tra)

    # translational probability green areas
    mean_green = 0
    std_green = 5
    norm_green = st.norm(loc=mean_green, scale=std_green)

    # rotational probability
    mean_rot = 0
    std_rot = 20
    norm_rot = st.norm(loc=mean_rot, scale=std_rot)

    # probability for blue objects appearance
    p = 0.5
    bernoulli_clutter = st.bernoulli(p)

    # Stage simulator map dimension
    stage_dim = 20

    translated_objs_image = image_objects_removed
    rectangles_info = []
    contours_green_translated = list(contours[colors.index("green")])
    green_idx = 0

    for j, (contour, obj_color_idx, object_image, movement_area) in enumerate(contour_obj_image_movement_area):
        if colors[obj_color_idx] == "green":
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

            # Convert to a Cartesian system with the origin in the center of stage simulator
            center_x = (stage_dim*center_x)/image.shape[1] - stage_dim / 2
            center_y = (stage_dim*-center_y)/image.shape[1] + stage_dim / 2

            # Convert pixel units to the desired unit (n pixels per unit)
            w = stage_dim*(w/image.shape[0])
            h = stage_dim*(h/image.shape[1])

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
        elif colors[obj_color_idx] == 'blue' and bernoulli_clutter.rvs():
            # If object is clutter and luck says to skip it
            #TODO: aggiungi log in vari punti, per esempio qui fai rospy.logdebug("Skipping blue object.")
            continue
        else:            
            translated_objs_image, _, _ = translate_obj(object_image, movement_area, translated_objs_image, dist_tra=norm_tra, dist_rot=norm_rot, show_steps=show_steps, disable_rotation=False)

    green_objs_translated = image_objects_removed.copy()
    green_objs_translated = cv2.drawContours(green_objs_translated, contours_green_translated, -1, (0, 255, 0), cv2.FILLED)

    # Display the original image and the result
    if show_recap:
        _ = plt.subplot(331), plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB)), plt.title('Original Image')
        _ = plt.subplot(334), plt.imshow(color_masks[0], cmap='gray'), plt.title("{} Pixels Mask".format(colors[0].title()))
        _ = plt.subplot(335), plt.imshow(color_masks[1], cmap='gray'), plt.title("{} Pixels Mask".format(colors[1].title()))
        _ = plt.subplot(336), plt.imshow(color_masks[2], cmap='gray'), plt.title("{} Pixels Mask".format(colors[2].title()))
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
            print("Saved rectangles info as {}".format(rectangles_path))

    return translated_objs_image

def check_positive(value):
    try:
        value = int(value)
        if value <= 0:
            raise argparse.ArgumentTypeError("{} is not a positive integer".format(value))
    except ValueError:
        raise Exception("{} is not an integer".format(value))
    return value

def check_positive_or_zero(value):
    try:
        value = int(value)
        if value < 0:
            raise argparse.ArgumentTypeError("{} is not a positive integer nor zero".format(value))
    except ValueError:
        raise Exception("{} is not an integer".format(value))
    return value


def parse_args():
    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()
    # get the file path for tirocinio
    package_path = rospack.get_path('tirocinio')

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
    parser.add_argument("-w", "--worlds", type=check_positive_or_zero, default=0, metavar="N",
        help="Use this to produce N maps and save them.")    
    parser.add_argument('--no-timestamp', action='store_true',
        help="""Use this save a single image without timestamp. If image.png already exists, it will create image_n.png
            in which n is the smallest number so that there is no file with that name. Same goes for the rectangles file.""")
    parser.add_argument("-d", '--dir', default=os.getcwd(),
        help="Base directory to save files in.")
    parser.add_argument('--steps', action='store_true',
        help="Use this to show the processing steps.")
    return parser.parse_args()

def get_world_text(image):
    return """
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
            range [ 0.06 4.095 ]
            fov 240.0
            samples 683

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

    quit_time 36000 # 10 hours of simulated time
    speedup 10

    paused 0

    resolution 0.01

    # configure the GUI window
    window
    (
        size [ 700.000 660.000 ] # in pixels (size [width height])
        scale 27.864467231386534  # pixels per meter
        center [ 0  0 ]
        rotate [ 0  0 ]
                    
        show_data 1              # 1=on 0=off
    )

    # load an environment bitmap
    floorplan
    ( 
        name  "turtlebot3-stage"
        size [ 20.0 20 1.0 ] # size [x y z]
        pose [0 0 0 0]""" + \
    """
        bitmap "bitmaps/image{}.png" # bitmap: solo il nero è renderizzato
    """.format(image) + \
    """
    )
    turtlebot3
    (		  
        # can refer to the robot by this name
        name  "turtlebot3"
        pose [ -5 4 0 0 ] 

        sick()
    )

    """


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


    movement_mask_image = cv2.imread(movement_mask_image_path, cv2.IMREAD_COLOR)
        
    image = cv2.imread(image_path, cv2.IMREAD_COLOR)

    if worlds == 0:
        if batch == 1:
            if no_timestamp:
                rectangles_path = os.path.join(base_dir, "rectangles.json")
            else:
                rectangles_path = os.path.join(base_dir, 'rectangles_' + str(time.time_ns()) + '.json')
            image_modified = extract_color_pixels(image, movement_mask_image, rectangles_path, show_recap=show_recap, show_steps=show_steps, save_map=save_map)

            if save_map:
                if no_timestamp:
                    filename = os.path.join(base_dir, "image.png")    
                    i = 1
                    while os.path.exists(filename): 
                        i += 1
                        filename = os.path.join(base_dir, "image_" + str(i) + ".png")
                else:
                    filename = os.path.join(base_dir, "image_" + str(time.time_ns()) + ".png")
                    while os.path.exists(filename): 
                        filename = os.path.join(base_dir, "image_" + str(time.time_ns()) + ".png")
                
                cv2.imwrite(filename, image_modified)
                print("Saved map as {}".format(filename))
        else:
            for i in range(batch):
                rectangles_path = os.path.join(base_dir, 'rectangles_' + str(i) + '.json')
                image_modified = extract_color_pixels(image, movement_mask_image, rectangles_path, show_recap=show_recap, show_steps=show_steps, save_map=True)
                filename = os.path.join(base_dir, "image_" + str(i) + ".png")
                cv2.imwrite(filename, image_modified)
                print("Saved map as {}".format(filename))
    else:
        for i in range(0, worlds):
            rectangles_path = os.path.join(base_dir, "bitmaps/rectangles{}.json".format(i))
            image_modified = extract_color_pixels(image, movement_mask_image, rectangles_path, show_recap=show_recap, show_steps=show_steps, save_map=True)
            filename = os.path.join(base_dir, "bitmaps/image{}.png".format(i))
            cv2.imwrite(filename, image_modified)
            print("Saved map as {}".format(filename))
            worldfile_path = os.path.join(base_dir, "world{}.world".format(i))
            with open(worldfile_path, "w", encoding="utf-8") as worldfile:
                worldfile.write(get_world_text(i))
                print("Saved worldfile as {}".format(worldfile_path))

if __name__ == "__main__":
    main()