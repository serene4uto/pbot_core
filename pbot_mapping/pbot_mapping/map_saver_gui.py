import rclpy
from rclpy.node import Node

import tkinter as tk
from ament_index_python.packages import get_package_share_directory
import os
import subprocess

class GuiMapSaver(tk.Tk, Node):

    def __init__(self):
        tk.Tk.__init__(self)
        Node.__init__(self, 'gui_map_saver')

        self.title("Map Saver")
        self.resizable(False, False)

        self.map_name_label = tk.Label(self, text="Map Name:")
        self.map_name_label.grid(row=0, column=0)
        self.map_name_textbox = tk.Entry(self, width=45)
        self.map_name_textbox.grid(row=0, column=1)
        self.map_name_textbox.insert(0, "scan_map")

        self.save_dir_label = tk.Label(self, text="Save Directory:")
        self.save_dir_label.grid(row=1, column=0)
        self.save_dir_textbox = tk.Entry(self, width=45)
        self.save_dir_textbox.grid(row=1, column=1)


        self.save_button = tk.Button(self, text="Save",
                                           command=self.save_map)
        self.save_button.grid(row=2, column=0, columnspan=2)


    
    def save_map(self):
        map_name = self.map_name_textbox.get()
        self.get_logger().info(f"Saving map as {map_name}")
        save_dir = self.save_dir_textbox.get()
        self.get_logger().info(f"Saving map in {save_dir}")
        try:
            self.run_ros2_map_saver(map_name, save_dir)
        except Exception as e:
            self.get_logger().error(e)
            self.get_logger().error("Failed to save map")
    
    def run_ros2_map_saver(self, map_name, save_dir):
        cmd = ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", os.path.join(save_dir, map_name)]
        subprocess.Popen(cmd)

def main(args=None):
    rclpy.init(args=args)

    gui_map_saver = GuiMapSaver()

    while rclpy.ok():
        rclpy.spin_once(gui_map_saver, timeout_sec=0.1)
        gui_map_saver.update()
    
