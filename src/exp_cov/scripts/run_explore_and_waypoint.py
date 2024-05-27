import subprocess as sp
from time import gmtime, strftime, sleep
import argparse
import cv2
from PIL import Image
import numpy as np
import os
import rospy

def parse_args():
    parser = argparse.ArgumentParser(description='Start exploration, logging time info to file and saving the final map.')
    parser.add_argument('--waypoints', required=True, help="Path to the waypoints csv file.")
    parser.add_argument('--world', required=True, help="Path to the stage world file.")
    parser.add_argument('-r', '--runs', required=False, default=1,  type=check_positive, help="Number of tests to run.", metavar="RUNS")
    return parser.parse_args()

def now():
    return strftime("%Y-%m-%d %H:%M:%S", gmtime())

def run_expl(logfile_path, run_subfolder = ""):   
    start = None
    args = ["roslaunch", "tirocinio", "explore_lite2.launch"]
    with sp.Popen(args, stdout=sp.PIPE, stderr=sp.STDOUT) as process:
        with open(logfile_path, mode="+a", encoding="utf-8") as logfile:
            try:
                start = rospy.get_rostime().secs
                logfile.write(f"{now()}: Starting exploration.\n")
                for line in process.stdout:
                    line = line.decode('utf8')
                    if line.strip()[1:].startswith("["):
                        if "exploration stopped." in line.lower():
                            logfile.write(f"{now()}: Finished exploration.\n")
                            break
            except KeyboardInterrupt as e:
                logfile.write(f"{now()}: Exploration Interrupted.\n")
            finally:
                time = rospy.get_rostime().secs - start
                logfile.write(f"{now()}: Exploration ros time is {strftime('%H:%M:%S', gmtime(time))}.\n")
                process.kill()
                map_name = os.path.join(run_subfolder, "Map_exploration")
                save_map = ["rosrun", "map_server", "map_saver", "-f", map_name]
                sp.run(save_map)
                try:
                    Image.open(f"{map_name}.pgm").save(f"{map_name}.png")
                    os.remove(f"{map_name}.pgm")
                except IOError:
                    print("Cannot convert pgm map to png.")
                finally:
                    return time

def run_cov(waypoints, logfile_path="./coverage.log", run_subfolder = ""):    
    start = None
    args = ["rosrun", "tirocinio", "waypoint_navigation.py", "-p", waypoints]
    with sp.Popen(args, stdout=sp.PIPE, stderr=sp.STDOUT) as process:
        with open(logfile_path, mode="+a", encoding="utf-8") as logfile:
            try:
                start = rospy.get_rostime().secs
                logfile.write(f"{now()}: Starting waypoint navigation.\n")
                for line in process.stdout:
                    line = line.decode('utf8')
                    if "final goal" in line.lower():
                        logfile.write(f"{now()}: Finished waypoint navigation.\n")
                        break
            except KeyboardInterrupt as e:
                logfile.write(f"{now()}: Waypoint navigation Interrupted.\n")
            finally:
                time = rospy.get_rostime().secs - start
                logfile.write(f"{now()}: Waypoint navigation ros time is {strftime('%H:%M:%S', gmtime(time))}.\n")
                process.kill()
                map_name = os.path.join(run_subfolder, "Map_coverage")
                save_map = ["rosrun", "map_server", "map_saver", "-f", map_name]
                sp.run(save_map)
                try:
                    Image.open(f"{map_name}.pgm").save(f"{map_name}.png")
                    os.remove(f"{map_name}.pgm")
                except IOError:
                    print("Cannot convert pgm map to png.")
                finally:
                    return time

def run_exploration(cmd_args, logfile_path, run_subfolder):
    print("starting exploration.")
    stage_args = ["roslaunch", "tirocinio", "stage_init.launch", f"worldfile:={cmd_args.world}"]
    slam_args = ["roslaunch", "tirocinio", "slam_toolbox_no_rviz.launch"]
    dist_args = ["rosrun", "tirocinio", "distance_check.py"]
    with sp.Popen(stage_args, stdout=sp.DEVNULL, stderr=sp.DEVNULL) as stage_process:
        sleep(3)
        print("started stage.")
        with sp.Popen(slam_args, stdout=sp.DEVNULL, stderr=sp.DEVNULL) as slam_process:
            sleep(10)
            print("started slam.")
            with open(logfile_path, mode="+a", encoding="utf-8") as logfile:
                with sp.Popen(dist_args, stdout=logfile, stderr=logfile) as dist_process:
                    time = run_expl(logfile_path, run_subfolder)
                    print("exploration finished.")
                    dist_process.terminate()
                    slam_process.terminate()
                    stage_process.terminate()
                    return time

def run_coverage(cmd_args, logfile_path, run_subfolder):
    print("starting coverage.")
    stage_args = ["roslaunch", "tirocinio", "stage_init.launch", f"worldfile:={cmd_args.world}"]
    slam_args = ["roslaunch", "tirocinio", "waypoint_slam.launch"]
    dist_args = ["rosrun", "tirocinio", "distance_check.py"]
    with sp.Popen(stage_args, stdout=sp.DEVNULL, stderr=sp.DEVNULL) as stage_process:
        sleep(3)
        print("started stage.")
        with sp.Popen(slam_args, stdout=sp.DEVNULL, stderr=sp.DEVNULL) as slam_process:
            sleep(10)
            print("started slam.")
            with open(logfile_path, mode="+a", encoding="utf-8") as logfile:
                with sp.Popen(dist_args, stdout=logfile, stderr=logfile) as dist_process:
                    time = run_cov(cmd_args.waypoints, logfile_path, run_subfolder)
                    print("coverage finished.")
                    dist_process.terminate()
                    slam_process.terminate()
                    stage_process.terminate()
                    return time

def check_positive(value):
    try:
        value = int(value)
        if value <= 0:
            raise argparse.ArgumentTypeError("{} is not a positive integer".format(value))
    except ValueError:
        raise Exception("{} is not an integer".format(value))
    return value

def main(cmd_args):    
    logfile_path_exploration = "explore.log"
    logfile_path_coverage = "coverage.log"
    logfile_path_result = "result.log"

    maxrun = 0
    for i in os.listdir(os.getcwd()):
        try:
            if int(i[3:]) >= maxrun:
                maxrun = int(i[3:]) + 1
        except:
            continue
    time_deltas = list()
    area_deltas = list()
    for r in range(int(cmd_args.runs)):
        print(f"run {r+1}/{cmd_args.runs} starting.")
        run_subfolder = f"run{maxrun+r}"
        os.mkdir(run_subfolder)
        logfile_path_exploration_run = os.path.join(run_subfolder, logfile_path_exploration)
        logfile_path_coverage_run = os.path.join(run_subfolder, logfile_path_coverage)
        logfile_path_result_run = os.path.join(run_subfolder, logfile_path_result)
        exploration_time = run_exploration(cmd_args, logfile_path_exploration_run, run_subfolder)
        sleep(2)
        coverage_time = run_coverage(cmd_args, logfile_path_coverage_run, run_subfolder)
        with open(logfile_path_result_run, mode="+a", encoding="utf-8") as logfile:
            time_delta = exploration_time-coverage_time
            time_deltas.append(exploration_time-coverage_time)
            msg = f"{now()}: Coverage time: {coverage_time}; Exploration time: {exploration_time}. Exploration - Coverage: {time_delta}. Unit is seconds."
            print(msg)
            logfile.write(f"{msg}\n")
            expl_map = cv2.imread(os.path.join(run_subfolder, "Map_exploration.png"), cv2.IMREAD_GRAYSCALE)
            cov_map = cv2.imread(os.path.join(run_subfolder, "Map_coverage.png"), cv2.IMREAD_GRAYSCALE)
            expl_map_area = np.sum(expl_map >= 250)
            cov_map_area = np.sum(cov_map >= 250)
            area_delta = expl_map_area-cov_map_area
            area_deltas.append(area_delta)
            msg = f"{now()}: Coverage mapped area: {cov_map_area}; Exploration mapped area: {expl_map_area}. Exploration - Coverage: {area_delta}. Unit is 0.05 meters, a pixel in the map."
            print(msg)
            logfile.write(f"{msg}\n")
        print(f"run {r+1}/{cmd_args.runs} finished.")
    print(f"time_deltas (exploration-coverage): {time_deltas}\n;\narea_deltas (exploration-coverage): {area_deltas}\n;\n")

if __name__ == "__main__":

    cmd_args = parse_args()
    with sp.Popen(["roscore"], stdout=sp.DEVNULL, stderr=sp.DEVNULL) as roscore_process:
        sleep(3)
        rospy.set_param('use_sim_time', True)
        rospy.init_node('just_for_time', anonymous=True)
        sleep(3)
        main(cmd_args)
        roscore_process.kill()

