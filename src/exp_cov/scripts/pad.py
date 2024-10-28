import cv2
import yaml
import os
import argparse

def parse_args():
    parser = argparse.ArgumentParser(description='Pad images.')
    parser.add_argument("-d", '--dir', default=os.getcwd(),
        help="Base directory for png and yaml files.")
    return parser.parse_args()

def main():
    borderType = cv2.BORDER_CONSTANT
    grey = [205]
    img_extensions = {".png"}
    yaml_extension = ".yaml"

    args = parse_args()
    base_dir = args.dir

    paths = []
    imgs = []
    for filepath in os.listdir(base_dir):
        filepath = os.path.join(base_dir, filepath)
        if os.path.splitext(filepath)[1] in img_extensions and os.path.exists(os.path.join(os.path.splitext(filepath)[0] + yaml_extension)):
            paths.append((filepath, os.path.join(os.path.splitext(filepath)[0] + yaml_extension)))

    for path in paths:
        img = cv2.imread(path[0], cv2.IMREAD_GRAYSCALE)
        info = None
        with open(path[1], 'r') as file:
            info = yaml.safe_load(file)
            imgs.append((img, info, path[0]))

    min_bottom_stage = 0
    min_left_stage = 0
    resolution = imgs[0][1]["resolution"]

    # Note: origin [x y z yaw]

    for img in imgs:
        height, width = img[0].shape[:2]
        min_left_stage = min(img[1]["origin"][0], min_left_stage)
        min_bottom_stage = min(img[1]["origin"][1], min_bottom_stage)

    max_height = 0
    max_width = 0

    for img in imgs:
        height, width = img[0].shape[:2]
        bottom = round(abs(min_bottom_stage - img[1]["origin"][1]) * (1 / resolution))
        new_height = height + bottom
        max_height = max(max_height, new_height)
        left = round(abs(min_left_stage - img[1]["origin"][0]) * (1 / resolution))
        new_width = width + left
        max_width = max(max_width, new_width)

    for img in imgs:
        height, width = img[0].shape[:2]
        bottom = round(abs(min_bottom_stage - img[1]["origin"][1]) * (1 / resolution))
        top = round(abs(max_height - (height + bottom)))
        left = round(abs(min_left_stage - img[1]["origin"][0]) * (1 / resolution))
        right = round(abs(max_width - (width + left)))
        padded = cv2.copyMakeBorder(img[0], top, bottom, left, right, borderType, None, grey)
        cv2.imwrite(os.path.join(base_dir, os.path.splitext(img[2])[0] + "_padded" + os.path.splitext(img[2])[1]), padded)

if __name__ == '__main__':
    main()