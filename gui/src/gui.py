import tkinter as tk
from PIL import Image, ImageTk
from threading import Thread
from multiprocessing import Process, Event

import rclpy

from .thrusters import thrusters
from .sliders import sliders
from .vitals import vitals
from .map import map
from .gui_node import guiNode
from .utils import display_camera, get_screens_info

class gui:
    """
    Class to create the GUI.

    Args:
        window(tk.Tk): The main window of the GUI.
        geometry(tuple): The geometry of the screen.

    Attributes:
        dir(str): The directory where the images are stored.
        shapes_bool(bool): Flag to start the shapes task.
        fish_bool(bool): Flag to start the fish task.
        line_bool(bool): Flag to start the line task.
        cams_switch(dict): The dictionary of the switch camera IPs.
        cam_ips(dict): The dictionary of the camera IPs.
        cam_pos(list): The list of camera positions.
        cam_size(list): The list of camera window sizes.
        cam_processes(list): The list of camera processes.
        stop_events(list): The list of stop events.
    """
    def __init__(self, 
                 window: tk.Tk, 
                 geometry: tuple) -> None:
        self.window = window
        self.geometry = geometry
        self.dir = "/home/atef/colcon_ws/src/gui/gui/gui_imgs"   # Change this to the directory where the images are stored

        # Variables
        self.shapes_bool = False
        self.fish_bool = False
        self.line_bool = False

        # Camera IPsx
        self.cams_switch = {
            "Tilt": "rtsp://192.168.1.120:8554/video2_unicast", 
            "Bottom": "rtsp://192.168.1.120:8554/video10_unicast"
        }

        self.cam_ips = {
            "Side": "rtsp://192.168.1.120:8554/video4_unicast",
            "Main": "rtsp://192.168.1.120:8554/video0_unicast", 
            "Gripper_L": "rtsp://192.168.1.120:8554/video6_unicast",
            "Gripper_R": "rtsp://192.168.1.120:8554/video8_unicast",
            "Switch": self.cams_switch["Tilt"]
        }

        # Camera positions and window sizes
        self.cam_pos = [(0, 0), (2 / 5, 0), (0, 1 / 2), (2 / 3, 1 / 2), (1 / 3, 1 / 2)]
        self.cam_size = [(2 / 5, 1 / 2), (3 / 5, 1 / 2), (1 / 3, 1 / 2), (1 / 3, 1 / 2), (1 / 3, 1 / 2)]

        # Processes and Events
        self.cam_processes = [0] * 5
        self.stop_events = [0] * 5

        self.create_widgets()
        self.init_cameras()

    def create_widgets(self) -> None:
        """Method to create the widgets of the GUI."""
        self.screen_width = self.geometry[0]
        self.screen_height = self.geometry[1]
        window_size = f"{self.screen_width}x{self.screen_height}+{self.geometry[2]}+{self.geometry[3]}"
        self.window.geometry(window_size)
        self.window.title("Shiro Kaijin Control Room")

        self.title_font = ("Garamond", 35, "bold")
        self.button_font = ("Garamond", 25, "bold")

        # Background Image
        self.bg = Image.open(
            f"{self.dir}/bg.jpeg"
        )
        self.bg = self.bg.resize((self.screen_width, self.screen_height))
        self.bg = ImageTk.PhotoImage(self.bg)

        self.back_canvas = tk.Canvas(
            self.window, width=self.screen_width, height=self.screen_height
        )
        self.back_canvas.create_image(0, 0, image=self.bg, anchor="nw")
        self.back_canvas.pack(fill="both", expand=True)

        # Title
        self.back_canvas.create_text(
            self.screen_width // 2, 150, text="Shiro Kaijin Control Room", font=self.title_font, 
            fill="#f8f8f8"
        )

        tk.Frame(self.back_canvas, bg="").grid(
            row=0, column=0, padx=10, pady=80, sticky="ew", columnspan=3
        )

        self.back_canvas.grid_columnconfigure(0, weight=1)
        self.back_canvas.grid_columnconfigure(1, weight=5)
        self.back_canvas.grid_columnconfigure(2, weight=5)

        self.column1 = tk.Frame(self.back_canvas, bg="")
        self.column1.grid(row=1, column=0, padx=(80, 10), pady=10, sticky="nsew")

        self.column2 = tk.Frame(self.back_canvas, bg="")
        self.column2.grid(row=1, column=1, padx=10, pady=10, sticky="nsew")

        self.column3 = tk.Frame(self.back_canvas, bg="")
        self.column3.grid(row=1, column=2, padx=(10, 80), pady=10, sticky="nsew")

        # Thrusters
        self.thrusters_gui = thrusters(self.column1, self.dir)

        # Sliders
        self.sliders_gui = sliders(self.column1)

        # ROV Vitals
        self.vitals_gui = vitals(self.column2)

        # Start Button
        self.start_button = tk.Button(
            self.column2, text="Start", font=self.button_font, bg="#ffffff", 
            command=self.start, height=2
        )
        self.start_button.grid(row=7, column=0, padx=10, pady=10, sticky="nsew", columnspan=2)

        # Buttons
        self.create_buttons_gui()

        # Map
        self.map_gui = map(self.column3, self.dir, self.screen_width)

    def create_buttons_gui(self) -> None:
        """Method to create the buttons GUI."""
        self.buttons_frame = tk.Frame(self.column3, bd=2, relief="groove", bg="")
        self.buttons_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")
        self.buttons_frame.grid_columnconfigure(0, weight=1)
        self.buttons_frame.grid_columnconfigure(1, weight=1)
        self.buttons_frame.grid_rowconfigure(0, weight=1)
        self.buttons_frame.grid_rowconfigure(1, weight=1)

        self.shapes_button = tk.Button(
            self.buttons_frame, text="Shapes", font=self.button_font, bg="#ffffff", 
            command=self.shapes, width=10, height=2
        )
        self.shapes_button.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        self.fish_button = tk.Button(
            self.buttons_frame, text="Fish", font=self.button_font, bg="#ffffff", 
            command=self.fish, width=10, height=2
        )
        self.fish_button.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")

        self.line_button = tk.Button(
            self.buttons_frame, text="Line", font=self.button_font, bg="#ffffff", 
            command=self.line, width=10, height=2
        )
        self.line_button.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")

        self.switch_cam_button = tk.Button(
            self.buttons_frame, text="Switch Cam", font=self.button_font, bg="#ffffff", 
            command=self.switch_cam, width=10, height=2
        )
        self.switch_cam_button.grid(row=1, column=1, padx=10, pady=10, sticky="nsew")

    def start(self) -> None:
        """Method to start the ROV.
        TODO: Add the start commands here."""
        pass

    def shapes(self) -> None:
        """Method to set the flag to start the shapes task."""
        self.shapes_bool = True

    def fish(self) -> None:
        """Method to set the flag to start the fish task."""
        self.fish_bool = True

    def line(self) -> None:
        """Method to set the flag to start the line task."""
        self.line_bool = True

    def switch_cam(self) -> None:
        """Method to switch the camera feed."""
        if self.cam_ips["Switch"] == self.cams_switch["Tilt"]:
            self.cam_ips["Switch"] = self.cams_switch["Bottom"]
        else:
            self.cam_ips["Switch"] = self.cams_switch["Tilt"]

        self.stop_events[-1].set()
        self.cam_processes[-1].terminate()
        self.stop_events[-1] = Event()
        self.cam_processes[-1] = Process(
            target=display_camera,
            args=(self.cam_ips["Switch"], "Switch", self.cam_size[-1], self.cam_pos[-1], 
                  get_screens_info(2), self.stop_events[-1]),
        )
        self.cam_processes[-1].start()  
        print("Switched to Camera IP:", self.cam_ips["Switch"])

    def init_cameras(self) -> None:
        """Method to initialize the camera feeds."""
        for i in range(len(self.cam_ips)):
            self.stop_events[i] = Event()
            process = Process(
                target=display_camera,
                args=(list(self.cam_ips.values())[i], list(self.cam_ips.keys())[i], 
                      self.cam_size[i], self.cam_pos[i], get_screens_info(2), self.stop_events[i]),
            )
            self.cam_processes[i] = process
            process.start()  

def ros_init(gui_obj: gui) -> None:
    """
    Function to initialize the ROS2 node.

    Args:
        gui_obj(gui): The GUI object.
    """
    rclpy.init(args=None)
    node = guiNode(gui_obj)
    rclpy.spin(node)
    rclpy.shutdown()

def main() -> None:
    """Function to initialize the GUI."""
    window = tk.Tk()
    gui_object = gui(window, get_screens_info(1))

    ros_thread = Thread(target=ros_init, args = (gui_object, ))
    ros_thread.start()

    window.mainloop()

if __name__ == "__main__":
    main()