import pyautogui
import time
import sys

# Function to center the mouse in a specified region
def center_mouse(x, y, width, height):
    center_x = x + width // 2
    center_y = y + height // 2
    pyautogui.moveTo(center_x, center_y)

# Function to drag the mouse left
def drag_left(distance, duration, button):
    pyautogui.drag(-distance, 0, duration=duration, button=button)

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
    click_point(x, y, click_duration)
    time.sleep(1)
    click_point(x, y, click_duration)
    press_f_key()

def disturb(duration):
    # Drag the mouse pointer left
    drag_left(200, duration, "left")
    drag_left(200, duration, "right")

def main():

    modes = ("select", "disturb", "full")
    maps_rgb = ("1", "2", "3")
    mode = "full"
    map_rgb = 1

    if len(sys.argv) > 2 and sys.argv[1] in modes and sys.argv[2] in maps_rgb:
        mode = sys.argv[1]
        map_rgb = sys.argv[2]

    # Window coordinates and size
    window_x = 904
    window_y = 421
    window_width = 700
    window_height = 660

    # Set the duration for the drag action (seconds)
    drag_duration = 2

    xs = (1105, 0, 0)
    ys = (610, 0, 0)
    click_duration = 0.3

    scroll_amount = 40

    try:
        if mode in ("select", "full"):
            select(xs[map_rgb], ys[map_rgb], click_duration)

            time.sleep(1)

        # Zoom in
        scroll_forward(scroll_amount)

        # Center the mouse pointer in the specified window
        center_mouse(window_x, window_y, window_width, window_height)

        # Pause for a moment (optional)
        time.sleep(1)

        if mode in ("disturb", "full"):
            disturb(drag_duration)
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == '__main__':
    main()