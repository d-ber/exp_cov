import cv2
import numpy as np
import matplotlib.pyplot as plt
import scipy.stats as st
import os
import time
import sys
import json
import geometry_msgs.msg as geo
import rospkg
import argparse

class Rectangle:
    def __init__(self, min_point=None, max_point=None, center=None, width=None, height=None):
        if min_point is not None and max_point is not None:
            # Using the original constructor with min_point and max_point
            self.min_point = min_point
            self.max_point = max_point
            self.calculate_center_dimensions()
        elif center is not None and width is not None and height is not None:
            # Using the new constructor with center_x, center_y, width, and height
            self.center = center
            self.width = width
            self.height = height
            self.calculate_min_max_points()
        else:
            raise ValueError("Invalid parameters. Please provide either min_point and max_point or center_x, center_y, width, and height.")

    def calculate_min_max_points(self):
        half_width = self.width / 2
        half_height = self.height / 2
        self.min_point = geo.Point(self.center.x - half_width, self.center.y - half_height, 0)
        self.max_point = geo.Point(self.center.x + half_width, self.center.y + half_height, 0)

    def calculate_center_dimensions(self):
        self.center = geo.Point((self.min_point[0] + self.max_point[0]) / 2, (self.min_point[1] + self.max_point[1]) / 2, 0)
        self.width = abs(self.max_point.x - self.min_point.x)
        self.height = abs(self.max_point.y - self.min_point.y)

# obj_img is a b&w image, in which the object is black and the background white
# img is an image
def translate_obj(obj_img, img, dx, dy):
    height, width = obj_img.shape[:2]

    # create the translation matrix using dx and dy, it is a NumPy array 
    translation_matrix = np.array([
        [1, 0, dx],
        [0, 1, dy]
    ], dtype=np.float32)
    translated_image = cv2.warpAffine(src=obj_img, M=translation_matrix, dsize=(width, height))
    translated_image = cv2.threshold(translated_image, 127, 255, cv2.THRESH_BINARY)[1]
    translated_image = cv2.bitwise_not(translated_image)

    #TODO: avoid writing and reading images
    cv2.imwrite("/tmp/image1.png", translated_image)
    cv2.imwrite("/tmp/image2.png", img)
    img1 = cv2.imread("/tmp/image1.png")
    img2 = cv2.imread("/tmp/image2.png")

    overlapping = cv2.bitwise_and(cv2.bitwise_not(img1), cv2.bitwise_not(img2)).any()
    #print(overlapping)
    dst = cv2.bitwise_and(img1,img2)
    return dst

# obj_img is a b&w image, in which the object is black and the background white
# img is an image
def translate_obj_show(obj_img, img, dx, dy):
    height, width = obj_img.shape[:2]

    # create the translation matrix using dx and dy, it is a NumPy array 
    translation_matrix = np.array([
        [1, 0, dx],
        [0, 1, dy]
    ], dtype=np.float32)
    translated_image = cv2.warpAffine(src=obj_img, M=translation_matrix, dsize=(width, height))
    translated_image = cv2.threshold(translated_image, 127, 255, cv2.THRESH_BINARY)[1]
    translated_image = cv2.bitwise_not(translated_image)

    #TODO: avoid writing and reading images
    cv2.imwrite("/tmp/image1.png", translated_image)
    cv2.imwrite("/tmp/image2.png", img)
    img1 = cv2.imread("/tmp/image1.png")
    img2 = cv2.imread("/tmp/image2.png")

    overlapping = cv2.bitwise_and(cv2.bitwise_not(img1), cv2.bitwise_not(img2)).any()
    #print(overlapping)
    dst = cv2.bitwise_and(img1,img2)
    if show_steps:
        _ = plt.subplot(221), plt.imshow(img1, cmap='gray'), plt.title('Pixels Mask')
        _ = plt.subplot(222), plt.imshow(img2, cmap='gray'), plt.title('Image')
        _ = plt.subplot(223), plt.imshow(dst), plt.title('Merged Image')
        plt.show()


def extract_color_pixels(image, color, show_steps=False, save_map=False):
    # Copy
    image_objects_removed = image.copy()
    
    # Convert RGB image to HSV (Hue, Saturation, Value) color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define the HSV range based on the specified color
    if color.lower() == 'red':
        lower_range = np.array([0, 100, 100])
        upper_range = np.array([10, 255, 255])
    elif color.lower() == 'green':
        lower_range = np.array([40, 40, 40])
        upper_range = np.array([80, 255, 255])
    elif color.lower() == 'blue':
        lower_range = np.array([100, 50, 50])
        upper_range = np.array([140, 255, 255])
    else:
        raise ValueError("Invalid color. Supported colors are 'red', 'green', or 'blue'.")

    # Create a binary mask for the specified color
    color_mask = cv2.inRange(hsv, lower_range, upper_range)

    # Find contours in the mask
    contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    image_with_boxes = cv2.cvtColor(color_mask, cv2.COLOR_GRAY2BGR)

    # Draw bounding boxes around each object
    for contour in contours:
        # Get the minimum area rectangle that bounds the contour
        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect)
        box = np.intp(box)

        # Draw the bounding box
        cv2.drawContours(image_with_boxes, [box], 0, (0, 255, 0), 2)  # Draw a green rectangle

    # Apply the mask to the original image
    result = cv2.bitwise_and(image_objects_removed, image_objects_removed, mask=color_mask)

    # Set the pixels in the original image where the color is extracted to white
    image_objects_removed[np.where(color_mask > 0)] = [255, 255, 255]

    # probability for red objects
    mean = 0
    standard_deviation = 10
    norm = st.norm(loc=mean, scale=standard_deviation)
    translations = norm.rvs(size=len(contours)*2)
    i = 0

    # probability for blue objects
    p = 0.5
    bernoulli = st.bernoulli(p)
    clutter_presence = bernoulli.rvs(size=len(contours))

    translated_objs_image = image_objects_removed

    rectangles_info = []

    for contour in contours:
        if color.lower() == "green":
            # Calculate the bounding rectangle
            x, y, w, h = cv2.boundingRect(contour)

            # Translate it 
            x = x + translations[0]
            y = y + translations[1]

            # Step 5: Convert coordinates to a Cartesian system
            center_x = x + w // 2
            center_y = y + h // 2

            # Convert to a Cartesian system with the origin in the center
            center_x -= image.shape[1] // 2  # Subtract half of the image width
            center_y = image.shape[0] // 2 - center_y  # Subtract half of the image height and invert y-axis

            # Convert pixel units to the desired unit (n pixels per unit)
            unit = 1/0.035888 #TODO: fattorizza in base al numero della mappa
            center_x /= unit
            center_y /= unit
            w /= unit
            h /= unit

            # Create a Rectangle instance
            rectangle = Rectangle(center = geo.Point(center_x, center_y, 0), width=w, height=h)

            # Add rectangle information to the list
            rectangles_info.append({
                "center": {
                    "x":rectangle.center.x,
                    "y":rectangle.center.y,
                    "z":rectangle.center.z},
                "width": rectangle.width,
                "height": rectangle.height
            })
        elif color.lower() == 'blue' and clutter_presence[i]:
            # If object is clutter and luck says to skip it
            continue
        
        # Create a mask for the current contour
        mask = np.zeros_like(color_mask)
        cv2.drawContours(mask, [contour], 0, 255, thickness=cv2.FILLED)

        # Extract the object using the mask
        object_image = cv2.bitwise_and(color_mask, color_mask, mask=mask)
        #print(translations[i*2], translations[(i*2)+1])
        translated_objs_image = translate_obj(object_image, translated_objs_image, translations[i*2], translations[(i*2)+1])
        i = i+1


    # Display the original image and the result
    if show_steps:
        _ = plt.subplot(231), plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB)), plt.title('Original Image')
        _ = plt.subplot(234), plt.imshow(color_mask, cmap='gray'), plt.title("{} Pixels Mask".format(color))
        _ = plt.subplot(235), plt.imshow(cv2.cvtColor(result, cv2.COLOR_BGR2RGB)), plt.title('Extracted {} Pixels'.format(color))
        _ = plt.subplot(236), plt.imshow(cv2.cvtColor(image_with_boxes, cv2.COLOR_BGR2RGB)), plt.title('Image with Bounding Boxes'.format(color))
        _ = plt.subplot(232), plt.imshow(cv2.cvtColor(image_objects_removed, cv2.COLOR_BGR2RGB)), plt.title('Without {} Pixels'.format(color))
        _ = plt.subplot(233), plt.imshow(cv2.cvtColor(translated_objs_image, cv2.COLOR_BGR2RGB)), plt.title('{} Objects translated'.format(color))
        plt.show()

    if color.lower() == 'green':
        if save_map:
            # Convert the list to JSON format
            json_data = json.dumps(rectangles_info, indent=4)

            # Write JSON data to a file
            path = 'rectangles_' + str(time.time_ns()) + '.json'
            with open(path, 'w') as json_file:
                json_file.write(json_data)
                print("Saved rectangles info as {}".format(path))

        return image_objects_removed
    else:
        return translated_objs_image

def parse_args():
    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()
    # get the file path for rospy_tutorials
    package_path = rospack.get_path('tirocinio')

    parser = argparse.ArgumentParser(description='Modify rgb maps automatically.')
    parser.add_argument('-m', '--map', default=os.path.join(package_path, "maps_rgb_lab/map1/map1_rgb.png"),
        help="Path to the rgb map file.", metavar="MAP_PATH")
    parser.add_argument('--show', action='store_true',
        help="Use this to show the processing steps.")
    parser.add_argument('--save', action='store_true',
        help="Use this to save the produced map and rectangles info.")
    return parser.parse_args()

def main():

    args = parse_args()
    image_path = args.map
    show_steps = args.show
    save_map = args.save
        
    image = cv2.imread(image_path, cv2.IMREAD_COLOR)
    image = extract_color_pixels(image, 'red', show_steps=show_steps, save_map=save_map) # oggetti semistatici
    image = extract_color_pixels(image, 'green', show_steps=show_steps, save_map=save_map) # aree di disturbo
    image = extract_color_pixels(image, 'blue', show_steps=show_steps, save_map=save_map) # clutter

    if save_map:
        filename = os.path.join(os.getcwd(), "image_" + str(time.time_ns()) + ".png")
        while os.path.exists(filename): 
            filename = os.path.join(os.getcwd(), "image_" + str(time.time_ns()) + ".png")
        cv2.imwrite(filename, image)
        print("Saved map as {}".format(filename))

    #TODO: estrarre tutti e 3 i colori insieme per evitare sovrapposizioni durante le varie estrazioni e spostamenti

if __name__ == "__main__":
    main()