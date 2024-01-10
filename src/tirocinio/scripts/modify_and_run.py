import subprocess as sp
import os
import argparse
import rospkg

def parse_args():
    parser = argparse.ArgumentParser(description='Modify a map and run stage on it.')
    parser.add_argument('--map', default=1, choices=[1, 2, 3], type=int,
        help="Number of rgb map.")
    return parser.parse_known_args()[0]


def main():
    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()
    # get the file path for tirocinio
    package_path = rospack.get_path('tirocinio')

    args = parse_args()
    map_rgb = str(args.map)
    map_rgb_path = os.path.join(package_path, "maps_rgb_lab/map" + map_rgb + "/map" + map_rgb + "_rgb.png")

    if os.path.exists(os.path.join(package_path, "world/bitmaps", "image.png")):
        os.remove(os.path.join(package_path, "world/bitmaps", "image.png"))
    if os.path.exists(os.path.join(package_path, "world/bitmaps", "rectangles.json")):
        os.remove(os.path.join(package_path, "world/bitmaps", "rectangles.json"))

    sp.run(["rosrun", "tirocinio", "map_rgb_simul.py", "--save", "--no-timestamp", "--map", map_rgb_path, "--dir", os.path.join(package_path, "world/bitmaps")])
    print(["rosrun", "tirocinio", "map_rgb_simul.py", "--save", "--no-timestamp", "--map", map_rgb_path, "--dir", os.path.join(package_path, "world/bitmaps")])

if __name__ == '__main__':
    main()
