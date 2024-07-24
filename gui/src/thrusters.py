import tkinter as tk
from PIL import Image, ImageTk


class thrusters(tk.Frame):
    """
    Class to create the thrusters frame.
    
    Inherits from:
        Frame (tk.Frame)
        
    Args:
        master(tk.Tk): The main window.
        dir(str): The directory where the images are stored.
    
    Attributes:
        thrusters(list): The list of thrusters values.
    """
    def __init__(self, 
                 master: tk.Frame,
                 dir: str) -> None:
        super().__init__(master)
        self.config(bd=2, relief="groove", bg="")
        self.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")
        
        self.thrusters = [0]*6
        self.dir = dir
        
        self.create_widgets()

    def create_widgets(self) -> None:
        """Method to create the thrusters widgets."""
        self.label_font = ("Garamond", 20, "bold")

        self.rov_image = Image.open(f"{self.dir}/rov.jpeg")
        self.rov_image = self.rov_image.resize((550, 350))
        self.rov_image = ImageTk.PhotoImage(self.rov_image)

        self.rov_canvas = tk.Canvas(
            self, width=self.rov_image.width(), height=self.rov_image.height()
        )
        self.rov_canvas.create_image(0, 0, image=self.rov_image, anchor="nw")
        self.rov_canvas.grid(
            row=0, column=0, padx=10, pady=10, sticky="nsew", columnspan=2
        )

        self.thrusters_labels = [0]*6
        self.thrusters_positions = [
            [440, 80],
            [50, 80],
            [50, 270],
            [440, 270],
            [245, 30],
            [245, 320]
        ]
        for i in range(6):
            self.thrusters_labels[i] = tk.Label(
                self.rov_canvas, text=str(self.thrusters[i]), font=self.label_font, bg="#ffffff"
            )
            self.thrusters_labels[i].place(
                x=self.thrusters_positions[i][0], y=self.thrusters_positions[i][1], anchor="center"
            )

    def update_thrusters(self) -> None:
        """Method to update the thrusters."""
        for i in range(6):
            self.thrusters_labels[i].config(text=str(self.thrusters[i]))