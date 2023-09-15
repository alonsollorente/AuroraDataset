#!/usr/bin/env python

#############################################################################
#                                                                           #
#   ROS GNSS 3D Position Plotter Script                                     #
#                                                                           #
#   Description: This script subscribes to the /gnss_pose topic, extracts   #
#   position data from incoming nav_msgs/Odometry messages, and plots       #
#   the positions in a 3D space using the matplotlib library.               #
#                                                                           #
#   Author: Alonso Llorente                                                 #
#   Date: 10-09-23                                                          #
#                                                                           #
#   Dependencies:                                                           #
#   - ROS (Robot Operating System)                                          #
#   - rospy library for ROS communication                                   #
#   - nav_msgs.msg for Odometry message handling                            #
#   - matplotlib library for data visualization                             #
#                                                                           #
#   Instructions:                                                           #
#   1. Replace 'your_bag_file.bag' with the path to your ROS bag file.      #
#   2. Run the script, and it will read the data from the bag file and      #
#      generate a 3D position plot with equally scaled axes and z-axis      #
#      limit between -10 and 100.                                           #
#                                                                           #
#############################################################################


import rosbag
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from nav_msgs.msg import Odometry

# Global variables to store position data
x_positions = []
y_positions = []
z_positions = []

bag_file_path = "/home/alonso/Desktop/gnss_pose_trial.bag"

def extract_data_from_bag(bag_file):
    global x_positions, y_positions, z_positions
    bag = rosbag.Bag(bag_file, 'r')
    
    for topic, msg, t in bag.read_messages(topics=['/gnss_pose']):
        # Extract position data
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        # Append data to global lists
        x_positions.append(x)
        y_positions.append(y)
        z_positions.append(z)
    
    bag.close()

def plot_3d_positions():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x_positions, y_positions, z_positions, label='Position')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('GNSS 3D Path')
    # ax.set_zlim(-10, 100)  # Set z-axis limit between 0 and 100
    plt.show()

if __name__ == '__main__':
    bag_file = bag_file_path  # Replace with your bag file path
    extract_data_from_bag(bag_file)
    
    # After reading data from the bag file, plot the 3D positions
    plot_3d_positions()