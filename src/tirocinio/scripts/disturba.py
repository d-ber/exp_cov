import pyautogui
import time

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

# Window coordinates and size
window_x = 904
window_y = 421
window_width = 700
window_height = 660

# Set the duration for the drag action (adjust as needed)
drag_duration = 2  # in seconds

x = 963
y = 510
duration = 0.3

scroll_amount = 40

try:
    click_point(x,y, duration)
    time.sleep(1)
    click_point(x,y, duration)
    press_f_key()

    time.sleep(1)

    # Drag the mouse pointer left
    scroll_forward(scroll_amount)

    # Center the mouse pointer in the specified window
    center_mouse(window_x, window_y, window_width, window_height)

    # Pause for a moment (optional)
    time.sleep(1)

    # Drag the mouse pointer left
    drag_left(200, drag_duration, "left")
    drag_left(200, drag_duration, "right")

except Exception as e:
    print(f"An error occurred: {e}")

finally:
    # Move the mouse pointer to a safe location (optional)
    pyautogui.moveTo(0, 0)
