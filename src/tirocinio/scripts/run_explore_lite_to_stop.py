import subprocess as sp
import os
import logging
from time import gmtime, strftime, sleep
import argparse
from PIL import Image
import rospy

def parse_args():
    parser = argparse.ArgumentParser(description='Start exploration, logging time info tofile and saving the final map.')
    parser.add_argument('-f', '--file', default="./exploration.log",
        help="Path to the log file.", metavar="LOG_FILE_PATH")
    return parser.parse_args()

def now():
    return strftime("%Y-%m-%d %H:%M:%S", gmtime())

def main(cmd_args):    
    start = None
    args = ["roslaunch", "tirocinio", "explore_lite2.launch"]
    with sp.Popen(args, stdout=sp.PIPE, stderr=sp.STDOUT) as process:
        with open(cmd_args.file, mode="+a", encoding="utf-8") as logfile:
            try:
                start = rospy.get_rostime().secs
                logfile.write(f"{now()}: Starting exploration.\n")
                for line in process.stdout:
                    line = line.decode('utf8')
                    if line.strip()[1:].startswith("["):
                        print(line)
                        if "exploration stopped." in line.lower():
                            logfile.write(f"{now()}: Finished exploration.\n")
                            break
            except KeyboardInterrupt as e:
                logfile.write(f"{now()}: Exploration Interrupted.\n")
            finally:
                print(f"{strftime('%H:%M:%S', gmtime(rospy.get_rostime().secs))}.\n")
                print(f"{strftime('%H:%M:%S', gmtime(start))}.\n")
                logfile.write(f"{now()}: Exploration ros time is {strftime('%H:%M:%S', gmtime(rospy.get_rostime().secs - start))}.\n")
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

