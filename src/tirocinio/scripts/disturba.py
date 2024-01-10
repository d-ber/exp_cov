import pyautogui
import time
import sys
import argparse
import scipy.stats as st

# Function to center the mouse in a specified region
def center_mouse(x, y, width, height):
    center_x = x + width // 2
    center_y = y + height // 2
    pyautogui.moveTo(center_x, center_y)

# Function to drag the mouse
def drag(distance, duration, button):
    pyautogui.drag(distance, 0, duration=duration, button=button)

# Function to click at a specific point with a specified duration
def click_point(x, y, duration):
    pyautogui.click(x, y, duration=duration)

# Function to press the "f" key
def press_f_key():
    pyautogui.press('f')

# Function to scroll the mouse wheel forward
def scroll_forward(scroll_amount):
    pyautogui.scroll(scroll_amount)

def select(x, y, click_duration):
    # Click to highlight window
    click_point(x, y, click_duration)
    # Sleep to give it some time
    time.sleep(1)
    # Click to select the robot
    click_point(x, y, click_duration)
    # F key to lock on the robot
    press_f_key()

def disturb(movement, rotation):
    # Drag the mouse pointer
    drag(movement, 1, "left") # To move the robot
    drag(rotation, 1, "right") # To rotate the robot

def parse_args():
    parser = argparse.ArgumentParser(description='"Disturb" a robot in stage simulator.')
    parser.add_argument('--map', default=1, choices=[1, 2, 3], type=int,
        help="Number of rgb map.")
    parser.add_argument('--mode', default="full", choices=["full", "disturb", "select"],
        help="Mode in which to launch the program. Select only selects the robot and zooms in. Disturb only distrubs it. Full does both.")
    return parser.parse_known_args()[0]

def main():

    args = parse_args()
    mode = args.mode
    map_rgb = args.map

    # Window coordinates and size
    #Lab:
    #window_x = 574
    #window_y = 196
    #Home:f
    window_x = 904
    window_y = 421
    window_width = 700
    window_height = 660

    # Click locations (xs[0] is for map_rgb number 1, etc...)f
    #Lab:
    #xs = (785, 0, 0)
    #ys = (430, 0, 0)
    #Home:
    xs = (1105, 0, 0)
    ys = (610, 0, 0)

    # Scroll amount to zoom in
    scroll_amount = 40
    # Select click duration
    click_duration = 0.3

    # Disturb random values (in pixels)
    mean = 0
    standard_deviation = 100
    norm = st.norm(loc=mean, scale=standard_deviation)
    disturb_vals = norm.rvs(size=2)
    drag_movement = disturb_vals[0]
    drag_rotation = disturb_vals[1]
    #print(disturb_vals)

    # Select the robot at saved coordinates
    if mode in ("select", "full"):
        select(xs[map_rgb-1], ys[map_rgb-1], click_duration)

        time.sleep(1)

        # Zoom in
        scroll_forward(scroll_amount)

    # Center the mouse pointer in the specified window
    center_mouse(window_x, window_y, window_width, window_height)
    pyautogui.click()

    # Pause for a moment
    time.sleep(1)

    # Randomly disturb robot
    if mode in ("disturb", "full"):
        disturb(drag_movement, drag_rotation)

if __name__ == '__main__':
    main()