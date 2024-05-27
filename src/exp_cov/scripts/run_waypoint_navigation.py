import subprocess as sp
import os
from time import gmtime, strftime, sleep
import argparse
from PIL import Image
import rospy

def parse_args():
    parser = argparse.ArgumentParser(description='Start waypoint navigation, logging time info to file and saving the final map.')
    parser.add_argument('-f', '--file', default="./coverage.log", help="Path to the log file.", metavar="LOG_FILE_PATH")
    parser.add_argument('-w', '--waypoints', required=True, help="Path to the waypoints csv file.")
    return parser.parse_args()

def now():
    return strftime("%Y-%m-%d %H:%M:%S", gmtime())

def run(waypoints, logfile_path="./coverage.log"):    
    start = None
    args = ["rosrun", "tirocinio", "waypoint_navigation.py", "-p", waypoints]
    with sp.Popen(args, stdout=sp.PIPE, stderr=sp.STDOUT) as process:
        with open(logfile_path, mode="+a", encoding="utf-8") as logfile:
            try:
                start = rospy.get_rostime().secs
                logfile.write(f"{now()}: Starting waypoint navigation.\n")
                for line in process.stdout:
                    line = line.decode('utf8')
                    if line.strip()[1:].startswith("["):
                        print(line)
                        if "final goal" in line.lower():
                            logfile.write(f"{now()}: Finished waypoint navigation.\n")
                            break
            except KeyboardInterrupt as e:
                logfile.write(f"{now()}: Waypoint navigation Interrupted.\n")
            finally:
                time = rospy.get_rostime().secs - start
                logfile.write(f"{now()}: Waypoint navigation ros time is {strftime('%H:%M:%S', gmtime(time))}.\n")
                process.kill()
                save_map = ["rosrun", "map_server", "map_saver", "-f", "Map"]
                sp.run(save_map)
                try:
                    Image.open("Map.pgm").save("Map.png")
                    os.remove("Map.pgm")
                except IOError:
                    print("Cannot convert pgm map to png.")
                finally:
                    return time
                    
if __name__ == "__main__":

    rospy.init_node('just_for_time', anonymous=True)
    sleep(3)
    args = parse_args()

    run(args.waypoints, args.file)

