import tkinter as tk
from paramiko import SSHClient

# Camera ports
CAM_PORTS = {
    "Main" : "/dev/video0", 
    "Tilt" : "/dev/video2", 
    "Side" : "/dev/video4", 
    "Gripper L" : "/dev/video6", 
    "Gripper R" : "/dev/video8", 
    "Bottom" : "/dev/video10"
}

CLIENT = SSHClient()
CLIENT.load_system_host_keys()

IP = "192.168.1.120"
AUTH = ("Jetson", "Nano")

class sliders(tk.Frame):
    """
    Class to create the sliders frame.

    Inherits from:
        Frame (tk.Frame)

    Args:
        master(tk.Tk): The main window.
    """
    def __init__(self, 
                 master: tk.Frame) -> None:
        super().__init__(master)

        self.config(bd=2, relief="groove", bg="")
        self.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")
        self.create_widgets()

    def create_widgets(self) -> None: 
        """Method to create the sliders widgets."""     
        self.label_font = ("Garamond", 20, "bold")
        self.small_button_font = ("Garamond", 10, "bold")

        self.cam_name = tk.StringVar()
        self.cam_name.set("Main")
        self.cam_menu = tk.OptionMenu(self, self.cam_name, *CAM_PORTS.keys())
        self.cam_menu.config(font=self.label_font, bg="#ffffff", width=8)
        self.cam_menu.grid(row=0, column=0, padx=10, pady=10, sticky="nse", columnspan=3)

        self.reset_cam_button = tk.Button(
            self, text="Reset All", font=self.label_font, bg="#ffffff", 
            command=self.reset_cameras,
        )
        self.reset_cam_button.grid(row=0, column=0, padx=10, pady=10, sticky="nsw")

        self.brightness = tk.Label(
            self, text="Brightness", font=self.label_font, bg="#ffffff", 
            width=10
        )
        self.brightness.grid(
            row=1, column=0, padx=10, pady=20, sticky="nsew"
        )

        self.brightness_slider = tk.Scale(
            self, orient="horizontal", from_=30, to=255
        )
        self.brightness_slider.grid(
            row=2, column=0, padx=10, pady=20, sticky="nsew"
        )
        self.brightness_slider.set(75)

        self.brightness_button = tk.Button(
            self,text="Change brightness", font=self.small_button_font, bg="#ffffff", 
            command=lambda: self.send_command(
                CAM_PORTS[self.cam_name.get()], "brightness", self.brightness_slider.get()
            )
        )
        self.brightness_button.grid(
            row=3, column=0, padx=10, pady=20, sticky="nsew"
        )

        self.contrast = tk.Label(
            self, text="Contrast", font=self.label_font, bg="#ffffff", 
            width=10
        )
        self.contrast.grid(
            row=1, column=1, padx=10, pady=20, sticky="nsew"
        )

        self.contrast_slider = tk.Scale(
            self, orient="horizontal", from_=0, to=10
        )
        self.contrast_slider.grid(
            row=2, column=1, padx=10, pady=20, sticky="nsew"
        )
        self.contrast_slider.set(5)

        self.contrast_button = tk.Button(
            self, text="Change contrast", font=self.small_button_font, bg="#ffffff", 
            command=lambda: self.send_command(
                CAM_PORTS[self.cam_name.get()], "contrast", self.contrast_slider.get()
            )
        )
        self.contrast_button.grid(
            row=3, column=1, padx=10, pady=20, sticky="nsew"
        )

        self.backlight = tk.Label(
            self, text="Backlight", font=self.label_font, bg="#ffffff", 
            width=10
        )
        self.backlight.grid(
            row=1, column=2, padx=10, pady=20, sticky="nsew"
        )

        self.backlight_slider = tk.Scale(
            self, orient="horizontal", from_=0, to=10
        )
        self.backlight_slider.grid(
            row=2, column=2, padx=10, pady=20, sticky="nsew"
        )
        self.backlight_slider.set(0)

        self.backlight_button = tk.Button(
            self, text="Change backlight", font=self.small_button_font, bg="#ffffff", 
            command=lambda: self.send_command(
                CAM_PORTS[self.cam_name.get()], "backlight_compensation", self.backlight_slider.get()
            )
        )
        self.backlight_button.grid(
            row=3, column=2, padx=10, pady=20, sticky="nsew"
        )

    @staticmethod
    def send_command(
        cam_port: str, 
        property: str, 
        value: int) -> None:
        """
        Send a command to modify the camera feed.
        
        Args:
            cam_port(str): The camera port.
            property(str): The property to change.
            value(int): The value to set the property to.
        """
        CLIENT.connect(IP, username=AUTH[0], password=AUTH[1])
        command = f"sudo v4l2-ctl -d {cam_port} -c {property}={value}"
        stdin, stdout, stderr = CLIENT.exec_command(command)

        _ = [print(i.strip()) for i in stdout]

        CLIENT.close()

    def reset_cameras(self) -> None:
        """Reset the cameras to their default settings."""
        reset_command = "source camera_reset.bash"
        CLIENT.connect(IP, username=AUTH[0], password=AUTH[1])
        stdin, stdout, stderr = CLIENT.exec_command(reset_command)

        _ = [print(i.strip()) for i in stdout]

        CLIENT.close()