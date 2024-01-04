import cv2
import numpy as np
import matplotlib.pyplot as plt

def extract_color_pixels(image_path, color):
    # Read the image in RGB format
    image = cv2.imread(image_path, cv2.IMREAD_COLOR)
    image_objects_removed = cv2.imread(image_path, cv2.IMREAD_COLOR)
    
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
        box = np.int0(box)

        # Draw the bounding box
        cv2.drawContours(image_with_boxes, [box], 0, (0, 255, 0), 2)  # Draw a green rectangle

    # Move each contour to the left by 10 pixels
    #for contour in contours:
    #    contour[:, :, 0] -= 10

    # Create an empty mask
    #shifted_mask = np.zeros_like(color_mask)

    # Draw the shifted contours on the mask
    #cv2.drawContours(shifted_mask, contours, -1, 255, thickness=cv2.FILLED)
    #cv2.drawContours(color_mask, contours, -1, (0,255,0), 3)

    # Apply the shifted mask to the original image
    #result = cv2.bitwise_and(image, image, mask=shifted_mask)

    # Apply the mask to the original image
    result = cv2.bitwise_and(image_objects_removed, image_objects_removed, mask=color_mask)

    # Set the pixels in the original image where the color is extracted to white
    image_objects_removed[np.where(color_mask > 0)] = [255, 255, 255]

    # Display the original image and the result
    _ = plt.subplot(231), plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB)), plt.title('Original Image')
    _ = plt.subplot(234), plt.imshow(color_mask, cmap='gray'), plt.title(f'{color.title()} Pixels Mask')
    _ = plt.subplot(235), plt.imshow(cv2.cvtColor(result, cv2.COLOR_BGR2RGB)), plt.title(f'Extracted {color.title()} Pixels')
    _ = plt.subplot(236), plt.imshow(cv2.cvtColor(image_with_boxes, cv2.COLOR_BGR2RGB)), plt.title('Image with Bounding Boxes')
    _ = plt.subplot(232), plt.imshow(cv2.cvtColor(image_objects_removed, cv2.COLOR_BGR2RGB)), plt.title(f'Without {color.title()} Pixels')
    plt.show()

image_path = '/home/d-ber/catkin_ws/src/tirocinio/maps_rgb_lab/map1/map1_rgb.png'
extract_color_pixels(image_path, 'red')
#extract_color_pixels(image_path, 'green')
#extract_color_pixels(image_path, 'blue')
