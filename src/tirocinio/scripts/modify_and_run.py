import subprocess as sp
import os
import argparse
import rospkg

def parse_args():
    parser = argparse.ArgumentParser(description='Modify a map and run stage on it.')
    parser.add_argument('--map', default=1, choices=[1, 2, 3], type=int,
        help="Number of rgb map.")
    parser.add_argument('--scan', default="/scan", help="Scan topic name to set for stage.")
    parser.add_argument('--screen', default="home", choices=["home", "lab"],
        help="Screen for the disturb node.")
    return parser.parse_known_args()[0]

def main():
    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()
    # get the file path for tirocinio
    package_path = rospack.get_path('tirocinio')

    args = parse_args()
    map_rgb = str(args.map)
    map_rgb_path = os.path.join(package_path, "maps_rgb_lab/map" + map_rgb + "/map" + map_rgb + "_rgb.png")
    movement_mask_path = os.path.join(package_path, "maps_rgb_lab/map" + map_rgb + "/map" + map_rgb + "_movement_mask.png")
    scan_topic = args.scan
    screen = args.screen

    image_path = os.path.join(package_path, "world/bitmaps", "image.png")
    rects_path = os.path.join(package_path, "world/bitmaps", "rectangles.json")
    world_path = os.path.join(package_path, "world/rgb.world")


    if os.path.exists(image_path):
        os.remove(image_path)
    if os.path.exists(rects_path):
        os.remove(rects_path)

    try:
        sp.run(["rosrun", "tirocinio", "map_rgb_simul.py", "--save", "--no-timestamp", "--map", map_rgb_path, "--mask", movement_mask_path,
            "--dir", os.path.join(package_path, "world/bitmaps")])

        sp.run(["roslaunch", "tirocinio", "stage_init_select.launch", "rectangles:=" + rects_path, 
        "map_rgb:=1", "worldfile:=" + world_path, "scan:=" + scan_topic, "screen:=" + screen])
    except KeyboardInterrupt:
        print("Exit request received. Closed.")

if __name__ == '__main__':
    main()
