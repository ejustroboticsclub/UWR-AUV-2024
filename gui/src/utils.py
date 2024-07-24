import cv2
from screeninfo import get_monitors
from typing import Tuple
from multiprocessing import Event

def display_camera(
        IP: str, 
        name: str, 
        size: Tuple[int, int], 
        position: Tuple[int, int], 
        screen: Tuple[int, int], 
        stop_event: Event
    ) -> None:
    """
    Function to display the camera feed.
    
    Args:
        IP(str): The IP address of the camera.
        name(str): The name of the window.
        size(tuple): The size of the window.
        position(tuple): The position of the window.
        screen(tuple): The screen information.
        stop_event(Event): Event to signal stopping the camera feed.
    """
    pipeline = "rtspsrc location=" + IP + " latency=0 buffer-mode=auto ! decodebin ! videoconvert ! appsink max-buffers=1 drop=True"
    
    w, h, x, y = screen
    cv2.namedWindow(name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(name, int(w*size[0]), int(h*size[1]))
    cv2.moveWindow(name, int(x+w*position[0]), int(y+h*position[1]))
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    while not stop_event.is_set():
        
        _,img = cap.read()
        if img is not None:
            cv2.imshow(name, img)
            
            if(cv2.waitKey(1) == ord('q')):
                break
        
    cap.release()
    cv2.destroyWindow(name)

def get_screens_info(screen: int) -> Tuple[int, int, int, int]:
    """
    Function to get the screen information.

    Args:
        screen(int): The screen number.

    Returns:
        tuple: The screen width, screen height, screen x-coordinate, and screen y-coordinate
    """
    monitors = get_monitors()
    if len(monitors) > 1 and screen == 2:
        screen = monitors[1]
    else:
        screen = monitors[0]
    return screen.width, screen.height, screen.x, screen.y