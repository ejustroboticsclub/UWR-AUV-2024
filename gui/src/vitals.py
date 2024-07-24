import tkinter as tk

class vitals(tk.Frame):
    """
    Class to create the ROV vitals frame.
    
    Inherits from:
        Frame (tk.Frame)
        
    Args:
        master(tk.Tk): The main window.

    Attributes:
        vx(float): The x velocity.
        vy(float): The y velocity.
        wz(float): The z angular velocity.
        roll(float): The roll angle.
        pitch(float): The pitch angle.
        yaw(float): The yaw angle.
        depth(float): The depth.
    """
    def __init__(self, 
                 master: tk.Frame) -> None:
        super().__init__(master)

        self.config(bd=2, relief="groove", bg="")
        self.grid(row=0, column=0, padx=10, pady=10, sticky="nsew", rowspan=2)

        self.vx = 0
        self.vy = 0
        self.wz = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.depth = 0

        self.create_widgets()

    def create_widgets(self) -> None:
        """Method to create the ROV vitals widgets."""      
        self.label_font = ("Garamond", 20, "bold")
        self.label_title_font = ("Garamond", 25, "bold")

        tk.Label(self, text="Vx", font=self.label_title_font, bg="#ffffff").grid(
            row=0, column=0, padx=10, pady=10, sticky="nsew"
        )
        self.vx_label = tk.Label(self, text=str(self.vx), font=self.label_font, 
                                 bg="#ffffff", width=10)
        self.vx_label.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")

        tk.Label(self, text="Vy", font=self.label_title_font, bg="#ffffff").grid(
            row=1, column=0, padx=10, pady=10, sticky="nsew"
        )
        self.vy_label = tk.Label(self, text=str(self.vy), font=self.label_font, 
                                 bg="#ffffff", width=10)
        self.vy_label.grid(row=1, column=1, padx=10, pady=10, sticky="nsew")

        tk.Label(self, text="Wz", font=self.label_title_font, bg="#ffffff").grid(
            row=2, column=0, padx=10, pady=10, sticky="nsew"
        )
        self.wz_label = tk.Label(self, text=str(self.wz), font=self.label_font, 
                                 bg="#ffffff", width=10)
        self.wz_label.grid(row=2, column=1, padx=10, pady=10, sticky="nsew")

        tk.Label(self, text="Roll", font=self.label_title_font, bg="#ffffff").grid(
            row=3, column=0, padx=10, pady=10, sticky="nsew"
        )
        self.roll_label = tk.Label(self, text=str(self.roll), font=self.label_font, 
                                   bg="#ffffff", width=10)
        self.roll_label.grid(row=3, column=1, padx=10, pady=10, sticky="nsew")

        tk.Label(self, text="Pitch", font=self.label_title_font, bg="#ffffff").grid(
            row=4, column=0, padx=10, pady=10, sticky="nsew", ipadx=15
        )
        self.pitch_label = tk.Label(self, text=str(self.pitch), font=self.label_font, 
                                    bg="#ffffff", width=10)
        self.pitch_label.grid(row=4, column=1, padx=10, pady=10, sticky="nsew")

        tk.Label(self, text="Yaw", font=self.label_title_font, bg="#ffffff").grid(
            row=5, column=0, padx=10, pady=10, sticky="nsew"
        )
        self.yaw_label = tk.Label(self, text=str(self.yaw), font=self.label_font, 
                                  bg="#ffffff", width=10)
        self.yaw_label.grid(row=5, column=1, padx=10, pady=10, sticky="nsew")

        tk.Label(self, text="Depth", font=self.label_title_font, bg="#ffffff").grid(
            row=6, column=0, padx=10, pady=10, sticky="nsew"
        )
        self.depth_label = tk.Label(self, text=str(self.depth), font=self.label_font, 
                                    bg="#ffffff", width=10)
        self.depth_label.grid(row=6, column=1, padx=10, pady=10, sticky="nsew")

    def update_vitals(self) -> None:
        """Method to update the ROV vitals."""
        self.vx_label.config(text=str(self.vx))
        self.vy_label.config(text=str(self.vy))
        self.wz_label.config(text=str(self.wz))
        self.roll_label.config(text=str(self.roll))
        self.pitch_label.config(text=str(self.pitch))
        self.yaw_label.config(text=str(self.yaw))
        self.depth_label.config(text=str(self.depth))