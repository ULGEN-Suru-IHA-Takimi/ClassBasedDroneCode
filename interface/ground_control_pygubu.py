import pathlib
import tkinter as tk
import tkinter.ttk as ttk
import pygubu
import time
import os
import sys
PROJECT_PATH = pathlib.Path(__file__).parent
PROJECT_UI = PROJECT_PATH / "ground_control.ui"
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from controllers.waypoint_controller import *
from controllers.xbee_controller import *


class Drone:
    def __init__(self):
        pass

class GroundControlApp:
    def __init__(self, master=None):


        
        self.waypoint = waypoints()
        self.drone_id = "0"

        self.builder = builder = pygubu.Builder()
        builder.add_resource_path(PROJECT_PATH)
        builder.add_from_file(PROJECT_UI)


        self.mainwindow = builder.get_object('mainwindow', master)
        self.mainwindow.protocol("WM_DELETE_WINDOW", self.on_closing)
        builder.connect_callbacks(self)

    def run(self):
        self.mainwindow.mainloop()

    def on_closing(self):
        self.mainwindow.destroy()
        try:
            self.xbee.disconnect()
        finally:
            exit()
        
    
    def connect_xbee(self):
        self.port = "/dev/ttyUSB0"
        try:
            self.xbee = XBeeController(port=self.port)
        except:
            pass



if __name__ == "__main__":
        root = tk.Tk()
        root.withdraw()
        app = GroundControlApp(root)
        app.run()
