import subprocess as sp
import os
import progressbar
import argparse

def make_map(args, world_num):
    image_path = args.map
    movement_mask_image_path = args.mask
    speedup = args.speedup
    pose = args.pose
    scale = args.scale

    sp.run(["python3", "/home/d-ber/catkin_ws/src/exp_cov/scripts/map_rgb_simul.py", "--map", image_path, "--mask", movement_mask_image_path,
        "--dir", os.path.join(os.getcwd(), "worlds"), "--speedup", str(speedup), "--pose", f"{pose[0]} {pose[1]}", "--scale", str(scale), "--world-num", str(world_num)])

    return(f"world{world_num}.world")

def main(args):
    mapName = make_map(args, 0)
    mapName = os.path.join(os.getcwd(), "worlds", str(mapName))
    rect_path = os.path.join(os.path.dirname(mapName), f"bitmaps/rectangles0.json")
    world_path = os.path.join(os.getcwd(), "worlds", str(mapName))
    
    print(["roslaunch", "exp_cov", "stage_init_select.launch", "worldfile:=" + world_path, "rectangles:=" + rect_path])
    sp.run(["roslaunch", "exp_cov", "stage_init_select.launch", "worldfile:=" + world_path, "rectangles:=" + rect_path])
    


def check_positive(value):
    try:
        value = int(value)
        if value <= 0:
            raise argparse.ArgumentTypeError("{} is not a positive integer".format(value))
    except ValueError:
        raise Exception("{} is not an integer".format(value))
    return value

def check_pose(value):
    try:
        ret_value = value.split()
        if len(ret_value) != 2:
            raise argparse.ArgumentTypeError(f"Given pose value \"{value}\" is not made of 2 numbers")
        return (float(ret_value[0]), float(ret_value[1]))
    except ValueError:
        raise Exception("{} is not made of 2 numbers".format(value))

def check_positive_float(value):
    try:
        value = float(value)
        if value <= 0:
            raise argparse.ArgumentTypeError("{} is not a positive float".format(value))
    except ValueError:
        raise Exception("{} is not a float".format(value))
    return value

def parse_args():
    parser = argparse.ArgumentParser(description='Start exploration in docker containers.')
    parser.add_argument('--map', default=os.path.join(os.getcwd(), "src/exp_cov/maps_rgb_lab/map1/map1_rgb.png"),
        help="Path to the rgb map file.", metavar="MAP_PATH")
    parser.add_argument('--mask', default=os.path.join(os.getcwd(), "src/exp_cov/maps_rgb_lab/map1/map1_movement_mask.png"),
        help="Path to the rgb mask map file for movement areas. Each rgb object in the map will be moved within the yellow mask given in this file. If the object is none, then it can move freely.", metavar="MASK_PATH")
    parser.add_argument("--speedup", type=check_positive, default=10, metavar="SPEEDUP",
        help="Use this to adjust stage simulation speed. Higher is faster but heavier on the CPU.") 
    parser.add_argument('--pose', type=check_pose, default=(0, 0), metavar="X Y",
        help="Robot pose X and Y coordinates.")
    parser.add_argument('--scale', type=check_positive_float, default=0.035888, metavar="PIXELS",
        help="Number of meters per pixel in png map.")
    return parser.parse_args()

if __name__ == "__main__":

    args = parse_args()

    main(args)

    print("All workers finished")


