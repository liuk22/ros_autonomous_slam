#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
import time 
import numpy as np
import os


ADJUSTED_MAP_BOUNDS = (50, 50, 300, 300) # x1, y1, x2, y2 of rectangular bounds of explorable indoor
STAGES = {0.001: False, 
    0.1 : False, 
    0.2 : False,
    0.3 : False, 
    0.4 : False, 
    0.5 : False, 
    0.6 : False, 
    0.7 : False, 
    0.8 : False, 
    0.9 : False}
TIME_LAST = 0
SAMPLING_PERIOD = 1
OCCUPIED_THRESH = 0.65
FREE_THRESH = 0.196
LOG_FOLDER_PATH = "/mnt/f/College/Research/Workspace/catkin_ws/src/ros_autonomous_slam/logs/"


def callback(message):
    if rospy.get_time() - TIME_LAST < SAMPLING_PERIOD:  
        return 
    map_grid = np.reshape(message.data, (message.info.height, message.info.width)).T
    x1, x2 = ADJUSTED_MAP_BOUNDS[0], ADJUSTED_MAP_BOUNDS[2]
    y1, y2 = ADJUSTED_MAP_BOUNDS[1], ADJUSTED_MAP_BOUNDS[3]
    map_grid_adjusted = map_grid[y1:y2, x1:x2]

    num_grid_squares = map_grid_adjusted.shape[0] * map_grid_adjusted.shape[1]
    fraction_mapped = 1 - (np.sum(map_grid_adjusted < 0) / num_grid_squares)
    
    global STAGES 
    found = False 
    for i, k in enumerate(sorted(list(STAGES.keys()))): 
        if not STAGES[k] and k <= fraction_mapped:
            STAGES[k] = True 
            log_file(map_grid_adjusted, i, k)
            found = True 
            break 
    if not found: 
        os.system("rosnode kill --all")

    global TIME_LAST
    TIME_LAST = rospy.get_time()


def log_file(grid_as_np, stage_num, stage_thresh): 
    named_tuple = time.localtime() # get struct_time
    log_name = "{}{}".format(LOG_FOLDER_PATH, time.strftime("%m-%d-%YT%H-%M-%S", named_tuple))
    log_name += "_stage{}_thresh{}.txt".format(stage_num, int(stage_thresh * 100))
    with open(log_name, 'w') as fh: 
        np.savetxt(fh, grid_as_np, fmt='%3.5f')


def node(): 
    rospy.init_node('exploration_progress', anonymous=True)
    
    
    map_topic = rospy.get_param("~map_topic", "map")
    global TIME_LAST
    TIME_LAST = rospy.get_time()
    rospy.Subscriber(map_topic, OccupancyGrid, callback)
    rospy.spin()


if __name__ == '__main__':
    node()