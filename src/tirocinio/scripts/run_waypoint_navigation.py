import subprocess as sp
import os
import logging
from time import gmtime, strftime, sleep
import argparse
from PIL import Image
import rospy

def parse_args():
    parser = argparse.ArgumentParser(description='Start waypoint navigation, logging time info to file and saving the final map.')
    parser.add_argument('-f', '--file', default="./exploration.log", help="Path to the log file.", metavar="LOG_FILE_PATH")
    parser.add_argument('-w', '--waypoints', required=True, help="Path to the waypoints csv file.")
    return parser.parse_args()

def now():
    return strftime("%Y-%m-%d %H:%M:%S", gmtime())

def main(cmd_args):    
    start = None
    args = ["rosrun", "tirocinio", "waypoint_navigation.py", "-p", cmd_args.waypoints]
    with sp.Popen(args, stdout=sp.PIPE, stderr=sp.STDOUT) as process:
        with open(cmd_args.file, mode="+a", encoding="utf-8") as logfile:
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
                logfile.write(f"{now()}: Waypoint navigation ros time is {strftime('%H:%M:%S', gmtime(rospy.get_rostime().secs - start))}.\n")
                process.kill()
                save_map = ["rosrun", "map_server", "map_saver", "-f", "Map"]
                sp.run(save_map)
                try:
                    Image.open("Map.pgm").save("Map.png")
                    os.remove("Map.pgm")
                except IOError:
                    print("Cannot convert pgm map to png.")
                    
if __name__ == "__main__":

    rospy.init_node('just_for_time', anonymous=True)
    sleep(3)
    args = parse_args()

    main(args)

