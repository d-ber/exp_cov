import os
import subprocess

# Get the current directory
current_dir = os.getcwd()

# List all directories in the current directory
directories = [d for d in os.listdir(current_dir) if os.path.isdir(os.path.join(current_dir, d)) and d.startswith("tesst")]

# Iterate over each directory and run prog.py if it exists
for directory in directories:
    print(f"Running prog in {directory}...")
    # Define the logfile name
    logfile = os.path.join(directory, "output.txt")
    # Run prog.py using subprocess with cwd parameter and output redirection
    with open(logfile, "w") as log:
        subprocess.run(['python3', '/home/d-ber/catkin_ws/src/exp_cov/scripts/run_explore_and_waypoint.py', 
        "--world", f"/home/d-ber/catkin_ws/src/exp_cov/tesi_testing2/{directory}/world0.world", "--waypoints", 
        "/home/d-ber/catkin_ws/src/exp_cov/tesi_testing2/poses.txt", "-r", "10"], 
        cwd=os.path.join(current_dir, directory), stdout=log, stderr=subprocess.STDOUT)

