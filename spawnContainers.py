import subprocess as sp
import os
from concurrent.futures import ThreadPoolExecutor
import progressbar
import argparse


def spawn_container(mapName: str, i, bar):
    bar.update(i)
    launchstr = f"""docker run -it \\
        --mount type=bind,source=./worlds,target=/root/catkin_ws/src/my_navigation_configs/worlds \\
        -v ./output:/root/catkin_ws/src/my_navigation_configs/runs/outputs \\
        'rosnoetic:explore' worlds/{mapName}"""
    p = sp.Popen(launchstr, shell=True, stdout=sp.DEVNULL)
    p.wait()

def purge_worlds():
    not_to_delete = ("rgb.world", "image.png", "rectangles.json")
    for root, _, files in os.walk("worlds/"):
        for file in files:
            if file not in not_to_delete:
                os.remove(os.path.join(root, file))

def main(workers: int):
    pool = ThreadPoolExecutor(max_workers=workers)
    try:
        with progressbar.ProgressBar(max_value=len(os.listdir("worlds/")) - 2) as bar:
            futures = []
            i = 1
            for name in os.listdir("worlds/"):
                if name.endswith(".world") and name != "rgb.world":
                    futures.append(pool.submit(spawn_container, name, i, bar))
                    i += 1
            pool.shutdown(wait=True)
    except Exception as e:
        print(e)
        pool.shutdown(wait=False)
        sp.Popen("docker kill $(docker ps -q)", shell=True)
        return
    finally:
        purge_worlds()

def check_positive(value):
    try:
        value = int(value)
        if value <= 0:
            raise argparse.ArgumentTypeError("{} is not a positive integer".format(value))
    except ValueError:
        raise Exception("{} is not an integer".format(value))
    return value

def parse_args():
    parser = argparse.ArgumentParser(description='Start exploration in docker containers.')
    parser.add_argument('--map', default=os.path.join(os.getcwd(), "src/tirocinio/maps_rgb_lab/map1/map1_rgb.png"),
        help="Path to the rgb map file.", metavar="MAP_PATH")
    parser.add_argument('--mask', default=os.path.join(os.getcwd(), "src/tirocinio/maps_rgb_lab/map1/map1_movement_mask.png"),
        help="Path to the rgb mask map file for movement areas. Each rgb object in the map will be moved within the yellow mask given in this file. If the object is none, then it can move freely.", metavar="MASK_PATH")
    parser.add_argument("--worlds", type=check_positive, default=1, metavar="WORLDS",
        help="Use this to produce WORLDS world files.")    
    parser.add_argument("--workers", type=check_positive, default=1, metavar="WORKERS",
        help="Use this to use WORKERS workers.")    
    return parser.parse_args()

if __name__ == "__main__":

    args = parse_args()
    image_path = args.map
    movement_mask_image_path = args.mask
    worlds = args.worlds
    workers = args.workers

    sp.run(["python3", os.path.join(os.getcwd(), "src/tirocinio/scripts/map_rgb_simul.py"), "--map", image_path, "--mask", movement_mask_image_path, "--worlds", str(worlds),
        "--dir", os.path.join(os.getcwd(), "worlds")])

    main(int(workers))

    print("All workers finished")
