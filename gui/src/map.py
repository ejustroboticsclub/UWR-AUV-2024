import tkinter as tk
from PIL import Image, ImageTk

class map(tk.Frame):
    """
    Class to create the map frame.

    Inherits from:
        Frame (tk.Frame)
    
    Args:
        master(tk.Tk): The main window.
        dir(str): The directory where the images are stored.
        screen_width(int): The width of the screen.
    
    Attributes:
        marks(list): The list of marks on the map.
    """
    def __init__(self, 
                 master: tk.Frame, 
                 dir: str, 
                 screen_width: int) -> None:
        super().__init__(master)

        self.config(bd=2, relief="groove", bg="")
        self.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")

        self.dir = dir
        self.screen_width = screen_width

        self.marks = []

        self.create_widgets()

    def create_widgets(self) -> None:   
        """Method to create the map widgets."""   
        self.label_font = ("Garamond", 20, "bold")
        self.label_title_font = ("Garamond", 25, "bold")

        self.map_buttons_frame = tk.Frame(self, bg="")
        self.map_buttons_frame.grid(row=0, column=0, pady=(160,10), sticky="e")

        self.delete_button = tk.Button(
            self.map_buttons_frame, text="Delete", font=self.label_font, bg="#ffffff", 
            command=self.delete_marks, width=5, height=1
        )
        self.delete_button.pack(side="left", padx=5, pady=5)

        self.undo_button = tk.Button(
            self.map_buttons_frame, text="Undo", font=self.label_font, bg="#ffffff", 
            command=self.undo_mark, width=5, height=1
        )
        self.undo_button.pack(side="left", padx=5, pady=5)

        self.map_image = Image.open(f"{self.dir}/map.jpeg")
        self.map_image = self.map_image.resize((self.screen_width // 3, 300))
        self.map_image = ImageTk.PhotoImage(self.map_image)

        self.map_canvas = tk.Canvas(
            self, width=self.screen_width // 3, height=300
        )
        self.map_canvas.create_image(0, 0, image=self.map_image, anchor="nw")
        self.map_canvas.grid(row=1, column=0, padx=10, pady=10, sticky="sew")
        self.map_canvas.bind("<Button-1>", self.draw_x)
        self.map_canvas.bind("<Button-3>", self.draw_o)

    def draw_x(self, 
               event: tk.Event) -> None:
        """
        Draw an 'X' on the canvas.
        
        Args:
            event(tk.Event): The event object.
        """
        x, y = event.x, event.y
        text_id = self.map_canvas.create_text(x, y, text="X", font=("Arial", 12, "bold"), fill="red")
        self.marks.append(("X", text_id, x, y))

    def draw_o(self, 
               event: tk.Event) -> None:
        """
        Draw an 'O' on the canvas.

        Args:
            event(tk.Event): The event object.
        """
        x, y = event.x, event.y
        text_id = self.map_canvas.create_text(x, y, text="O", font=("Arial", 12, "bold"), fill="blue")
        self.marks.append(("O", text_id, x, y))

    def delete_marks(self) -> None:
        """Delete all marks on the map."""
        for mark in self.marks:
            self.map_canvas.delete(mark[1])
        self.marks = []

    def undo_mark(self) -> None:
        """Undo the last mark placed on the map."""
        if self.marks:
            last_mark = self.marks.pop()
            self.map_canvas.delete(last_mark[1])