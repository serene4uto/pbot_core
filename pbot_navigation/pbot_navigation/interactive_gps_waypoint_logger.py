import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from pbot_navigation.utils.gps_utils import latLonYaw2Geopose
import tkinter as tk
from tkinter import messagebox
from sensor_msgs.msg import NavSatFix
import os
import sys
import yaml




class InteractiveGpsGuiLogger(tk.Tk, Node):
    def __init__(self, logging_file_path):
        tk.Tk.__init__(self)
        Node.__init__(self, 'interactive_gps_waypoint_logger')


        self.title("Interactive GPS Waypoint Logger")

        self.logging_file_path = logging_file_path

        self.gps_pose_label = tk.Label(self, text="Current Coordinates:")
        self.gps_pose_label.pack()
        self.gps_pose_textbox = tk.Label(self, text="", width=45)
        self.gps_pose_textbox.pack()
        self.listbox = tk.Listbox(self, width=45)
        self.listbox.pack()
        self.save_button = tk.Button(self, text="Save",
                                           command=self.save_listbox_to_file)
        self.save_button.pack()

        self.clear_button = tk.Button(self, text="Clear",
                                           command=self.clear_listbox)
        self.clear_button.pack()

        self.mapviz_wp_sub = self.create_subscription(
            PointStamped, "/clicked_point", self.mapviz_wp_cb, 1)
        
        self.selected_wps_pub = self.create_publisher(
            NavSatFix, "/igw_gps_points", 1)
        
        self.last_gps_position = NavSatFix()
        self.last_heading = 0.0
    
    def mapviz_wp_cb(self, msg: PointStamped):
        if msg.header.frame_id != "wgs84":
            self.get_logger().warning(
                "Received point from mapviz that ist not in wgs84 frame. This is not a gps point and wont be followed")
            return
        
        # self.last_gps_position = msg
        # self.updateTextBox()
        
        self.get_logger().info(f"Received point from mapviz at lon: {msg.point.x}, lat: {msg.point.y}")
        
        self.last_gps_position.latitude = msg.point.y
        self.last_gps_position.longitude = msg.point.x
        self.selected_wps_pub.publish(self.last_gps_position)

        self.listbox.insert(tk.END, f"{msg.point.y:.6f},{msg.point.x:.6f}") # lat, lon


    def updateTextBox(self):
        """
        Function to update the GUI with the last coordinates
        """
        self.gps_pose_textbox.config(
            text=f"Lat: {self.last_gps_position.latitude:.6f}, Lon: {self.last_gps_position.longitude:.6f}, yaw: {self.last_heading:.2f} rad")
        
    def clear_listbox(self):
        self.listbox.delete(0, tk.END)
    
    def save_listbox_to_file(self):

        listbox_data = self.listbox.get(0, tk.END)
        waypoint_data_list = {"waypoints": []}

        with open(self.logging_file_path, "w") as f:
            for line in listbox_data:
                data_line = line.split(",")
                data = {
                    "latitude": float(data_line[0]),
                    "longitude": float(data_line[1]),
                    "yaw": self.last_heading
                }
                waypoint_data_list["waypoints"].append(data)
            yaml.dump(waypoint_data_list, f)
        messagebox.showinfo("Saved", f"Saved waypoints to {self.logging_file_path}")


        
        


def main(args=None):
    rclpy.init(args=args)

    # allow to pass the logging path as an argument
    default_yaml_file_path = os.path.expanduser("gps_waypoints.yaml")
    if len(sys.argv) > 1:
        yaml_file_path = sys.argv[1]
    else:
        yaml_file_path = default_yaml_file_path

    igps_gui_logger = InteractiveGpsGuiLogger(yaml_file_path)

    while rclpy.ok():
        # we spin both the ROS system and the interface
        rclpy.spin_once(igps_gui_logger, timeout_sec=0.1)  # Run ros2 callbacks
        igps_gui_logger.update()  # Update the tkinter interface

    rclpy.shutdown()


if __name__ == '__main__':
    main()
    