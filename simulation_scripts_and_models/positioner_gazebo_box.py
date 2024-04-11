import cv2 
from time import sleep
import numpy as np
import matplotlib.pyplot as plt
from geopy import distance
from ultralytics import YOLO
import torch

import kalman
from  boat_positioning import bbox_centerangle, boat_angledim_compensator, boat_cam_angle, boat_distance_gazebo, bbox_centerangle_width

from pymavlink import mavutil

from gz.msgs10.image_pb2 import Image
# from gz.msgs10.annotated_axis_aligned_2d_box_pb2 import AnnotatedAxisAligned2DBox
from gz.msgs10.annotated_axis_aligned_2d_box_v_pb2 import AnnotatedAxisAligned2DBox_V
from gz.transport13 import Node

print(torch.cuda.is_available())

# Drone connection
drone_connection = mavutil.mavlink_connection('udpin:localhost:14551')
boat_connection = mavutil.mavlink_connection('udpin:localhost:14561') 


y_model = YOLO('yolov8n.pt')
boat_dims = [13.8, 4.4] # vessel A: ca [13.8, 4.4], SSRS Mercedes: [14.2, 4.2]
r_earth = 6378137 # m
image_pos = []
image_pos_old = image_pos
clear_var = 0 # just to get decent plots

Camera_matrix = [[1.1088e+03, 0.00000000e+00, 640],[0.00000000e+00, 1.1088e+03, 360],[0.00000000e+00, 0.00000000e+00, 1.00000000e+00]] # denna borde vara för den simulerade kameran nu men borde nog äkna om focal lenth flr y om jag inte skippar den helt(bara kollar i breddled)
Camera_matrix = np.array(Camera_matrix) # adjusting due to calibration being done in 1080p 1080p =*1, 4k = *2 ,720p = /1.5
camera_params = (Camera_matrix[0,0], Camera_matrix[1,1], Camera_matrix[0,2], Camera_matrix[1,2])

n = 4
m = 2 

dt = 0.2 # denna får vi ta fram
xk = np.array([[57.672,0,0,0],
            [0,11.841,0,0],
            [0,0,0,0],
            [0,0,0,0]])    # [n x 1] Prior mean 
P = np.eye(n)     # [n x n] Prior covariance

A = np.array([[1,dt, 0, 0],         
             [0, 1, 0, 0],
             [1, dt, 0, 0],
             [0, 1, 0, 0]]) # [n x n] State transition matrix 
Q =  np.eye(n)*0.05         # [n x n] Process noise covariance
H =   np.eye(n)             # [m x n] Measurement model matrix
R = np.eye(n)*0.1          # [m x m] Measurement noise covariance
y0k = xk 

def on_new_boxes_image(msg: Image):
    data = np.frombuffer(msg.data, dtype=np.uint8)
    RGB_img = data.reshape((msg.height, msg.width, 3))
    global img # kom inte på ett bättre sätt att göra detta på
    img = cv2.cvtColor(RGB_img, cv2.COLOR_RGB2BGR)
    return img

def on_new_boxes(msg: AnnotatedAxisAligned2DBox_V): # boxes message 
    # print(msg.annotated_box)
    if msg.annotated_box and msg.annotated_box[0].label == 8:
        # if msg.annotated_box[0].label == 8
        global image_pos
        min_corner = np.array([msg.annotated_box[0].box.min_corner.x, msg.annotated_box[0].box.min_corner.y])
        max_corner = np.array([msg.annotated_box[0].box.max_corner.x, msg.annotated_box[0].box.max_corner.y])

        bbox_center = (min_corner+max_corner)/2
        # print(bbox_center)
        # bbox_cent_offset = bbox_centerangle(bbox_center, Camera_matrix) # denna funkar inte just nu
        bbox_cent_offset = 0 
        print(bbox_cent_offset)
        print("======================================")
        # bbox_cent_offset = width_centerangle(bbox_center, Camera_matrix)
        # bbox_cent_offset = 0 

        # We must get the bearing data from the drone and also its current position
        # effective_len = boat_angledim_compensator(boat_dims, bbox_cent_offset, b_bearing_interpolated[frame_number], heading, relative_bearing)
        effective_len = boat_angledim_compensator(boat_dims, bbox_cent_offset, boat_heading, heading)
        # effective_len = boat_dims[1]
        b_dist = boat_distance_gazebo(effective_len, Camera_matrix, max_corner, min_corner) 

        image_pos = distance.distance(meters=b_dist).destination((d_lat,d_long),heading+bbox_cent_offset)
        
        return image_pos
    # print(msg.annotated_box[0].box.min_corner.x)
# Initialize the Gazebo node
node = Node() # image node
node_box = Node() # bounding box node

# Subscribing to the camera topic
node.subscribe(Image, "/boxes_image", on_new_boxes_image)
node_box.subscribe(AnnotatedAxisAligned2DBox_V, "boxes", on_new_boxes)
sleep(3) # Waiting for img to be defined

print("Waiting for heartbeat from drone")
drone_connection.wait_heartbeat()
boat_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (drone_connection.target_system, drone_connection.target_component))

AHRS_boat = boat_connection.recv_match(type = 'AHRS2', blocking = True)
vfr_boat = boat_connection.recv_match(type = 'VFR_HUD', blocking = True)
boat_lat, boat_long, boat_heading = AHRS_boat.lat/1e7, AHRS_boat.lng/1e7, vfr_boat.heading

msg_drone = drone_connection.recv_match(type = 'AHRS2', blocking = True)
vfr_msg = drone_connection.recv_match(type = 'VFR_HUD', blocking = True)
msg = drone_connection.recv_match(type='HOME_POSITION', blocking=False) # This probably stops it when starting the script after the simulation
d_lat, d_long = msg_drone.lat/1e7, msg_drone.lng/1e7
heading = vfr_msg.heading

drone_connection.mav.command_int_send(drone_connection.target_system, drone_connection.target_component, 0, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 242, 0, 0, 0, 0, 0, 0, 0, 1)

print("Entering main loop")

try:
    while True:

        # drone_connection.mav.command_int_send(drone_connection.target_system, drone_connection.target_component, 0, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 242, 0, 0, 0, 0, 0, 0, 0, 1)
        msg_drone = drone_connection.recv_match(type = 'AHRS2', blocking = False) # I think AHRS2 is the output from the EKF: https://discuss.ardupilot.org/t/scaled-imu-and-ahrs-differences/19949
        
        vfr_msg = drone_connection.recv_match(type = 'VFR_HUD', blocking = False)
        if msg_drone is not None: 
            d_lat, d_long = msg_drone.lat/1e7, msg_drone.lng/1e7 # drones position
        if vfr_msg  is not None:
            heading = vfr_msg.heading

        if image_pos != image_pos_old: # update the image position when we get a new (different) meassurement value
            # Filtered position
            yk = np.array([image_pos.latitude, image_pos.longitude, 0,0]).transpose() # antar att jag borde byta ut 0,0 mot någon långvarig medelhastighet på båten
            # print(yk)
            # print(xk)
            xk, P = kalman.kalman_filter(yk, xk, P, A, Q, H, R)
            
            image_pos_old = image_pos
            drone_connection.mav.command_int_send(drone_connection.target_system, drone_connection.target_component, 0, mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 0, 0, 0, 0, 0, int(xk[0,0]*1e7), int(xk[1,1]*1e7), 575) # kalman filtered
            
            # Plotting
            plt.scatter(xk[1,1], xk[0,0], color='orange', label='filtered pos') # filtered position
            plt.scatter(d_long, d_lat, color='red',label='drone pos') # drone position
            plt.scatter(boat_long, boat_lat, color = 'blue', label='boat pos') # boat position
            plt.draw()
            plt.pause(0.00001)
            clear_var += 1
            if clear_var % 100 == 0:
                plt.clf()
            if clear_var % 100 == 1:
                plt.legend()
            



            # drone_connection.mav.command_int_send(drone_connection.target_system, drone_connection.target_component, 0, mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 0, 0, 0, 0, 0, int(yk[0,0]*1e7), int(yk[0,1]*1e7), 575) # raw meassurements

        msg = drone_connection.recv_match(type='HOME_POSITION', blocking=False)
        if msg is not None:  
            home_lat_long_alt = [msg.latitude, msg.longitude, msg.altitude]

        cv2.imshow("image", img)
        cv2.waitKey(1)

        sleep(0.1) # to slow down the loop
        pass
except KeyboardInterrupt:
    pass










