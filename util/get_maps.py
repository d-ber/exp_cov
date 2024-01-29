import os
from os.path import isdir 
import sys
import shutil


if len(sys.argv) == 2:
    base_dir = sys.argv[1]
    result_dir = os.path.join(os.getcwd(), "Final_images")
    if not os.path.exists(result_dir):
        os.makedirs(result_dir)
    for dirpath, dirnames, filenames in os.walk(base_dir):
        for filename in filenames:
            if filename == "Map.png":
                world = os.path.basename(os.path.split(os.path.split(dirpath)[0])[0])
                run = os.path.basename(os.path.split(dirpath)[0])
                new_filename = os.path.join(result_dir, world, run + ".png")
                if not os.path.exists(os.path.dirname(new_filename)):
                    os.makedirs(os.path.dirname(new_filename))
                #print(f"from {os.path.join(dirpath, filename)} to {new_filename}")
                shutil.copy(os.path.join(dirpath, filename), new_filename)

else:
    print("Argument missing or too many given. Aborting.")