import cv2 
from time import sleep
import numpy as np
import matplotlib.pyplot as plt
from geopy import distance
from ultralytics import YOLO
import torch

import kalman
from  boat_positioning import bbox_centerangle, boat_angledim_compensator, boat_cam_angle, boat_distance

from pymavlink import mavutil

from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node

print(torch.cuda.is_available())

# Drone connection
the_connection = mavutil.mavlink_connection('udpin:localhost:14551') 

y_model = YOLO('yolov8n.pt')
boat_dims = [14.2, 4.2]
r_earth = 6378137 # m

Camera_matrix = [[1.31754161e+03, 0.00000000e+00, 1.01639924e+03],[0.00000000e+00, 1.31547107e+03, 5.24436193e+02],[0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]
Camera_matrix = np.array(Camera_matrix)*2 # adjusting due to calibration being done in 1080p 1080p =*1, 4k = *2 ,720p = /1.5
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
R = np.eye(n)*0.5          # [m x m] Measurement noise covariance
y0k = xk 

def on_image_frame(msg: Image):
    # Convert the string data to a NumPy array
    data = np.frombuffer(msg.data, dtype=np.uint8)
    RGB_img = data.reshape((msg.height, msg.width, 3))
    global img # kom inte på ett bättre sätt att göra detta på
    img = cv2.cvtColor(RGB_img, cv2.COLOR_RGB2BGR)
    return img

# Initialize the Gazebo node
node = Node()
# Subscribing to the camera topic
node.subscribe(Image, "/camera", on_image_frame)
sleep(3) # Waiting for img to be defined

print("Waiting for heartbeat from drone")
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

msg_drone = the_connection.recv_match(type = 'AHRS2', blocking = True)
vfr_msg = the_connection.recv_match(type = 'VFR_HUD', blocking = True)
msg = the_connection.recv_match(type='HOME_POSITION', blocking=True)
d_lat, d_long = msg_drone.lat/1e7, msg_drone.lng/1e7
heading = vfr_msg.heading

print("Entering main loop")
try:
    while True:

        the_connection.mav.command_int_send(the_connection.target_system, the_connection.target_component, 0, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 242, 0, 0, 0, 0, 0, 0, 0, 1)
        msg_drone = the_connection.recv_match(type = 'AHRS2', blocking = False) # I think AHRS2 is the output from the EKF: https://discuss.ardupilot.org/t/scaled-imu-and-ahrs-differences/19949
        
        vfr_msg = the_connection.recv_match(type = 'VFR_HUD', blocking = False)
        if msg_drone is not None: 
            d_lat, d_long = msg_drone.lat/1e7, msg_drone.lng/1e7
        if vfr_msg  is not None:
            heading = vfr_msg.heading

        results = y_model(img, stream=True, verbose=False,) # Verbose toggels outprint to console
        for result in results:
            im_array = result.plot()
        if result.boxes: # Right now we are positioning home position after all objects found
            print("==================================FOUND BOUNDING BOX==============================")
            img = im_array
            bbox_center = int((result.boxes.xyxy[0][1]+result.boxes.xyxy[0][3])/2), int((result.boxes.xyxy[0][0]+result.boxes.xyxy[0][2])/2) # egentligen kan jag byta ordning på dessa så att det blir [y,x] men gör det i funktionen bbox_centerangle nu
            bbox_cent_offset = bbox_centerangle(bbox_center, Camera_matrix)

            # We must get the bearing data from the drone and also its current position
            # effective_len = boat_angledim_compensator(boat_dims, bbox_cent_offset, b_bearing_interpolated[frame_number], heading, relative_bearing)
            effective_len = boat_dims[1]
            b_dist = boat_distance(effective_len, Camera_matrix, result.boxes) 

            yolo_dist = distance.distance(meters=b_dist).destination((d_lat,d_long),heading+bbox_cent_offset)
            # Unfiltered position
            # yolo_dist = distance.distance(meters=b_dist).destination((d_lat[frame_number],d_long[frame_number]),relative_bearing)
            # yolo_lat, yolo_long = yolo_dist.latitude, yolo_dist.longitude

            # Filtered position
            yk = np.array([yolo_dist.latitude, yolo_dist.longitude, 0,0]).transpose() # antar att jag borde byta ut 0,0 mot någon långvarig medelhastighet på båten
            xk, P = kalman.kalman_filter(yk, xk, P, A, Q, H, R)
            
            plt.scatter(yolo_dist.longitude, yolo_dist.latitude ,color='purple', label='Approx pos') # approx position  
            plt.scatter(xk[1,1], xk[0,0], color='orange', label='filtered pos') # filtered position

            the_connection.mav.command_int_send(the_connection.target_system, the_connection.target_component, 0, mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 0, 0, 0, 0, 0, int(xk[0,0]*1e7), int(xk[1,1]*1e7), 575)
            msg = the_connection.recv_match(type='HOME_POSITION', blocking=False)
            if msg is not None:  
                home_lat_long_alt = [msg.latitude, msg.longitude, msg.altitude]

        cv2.imshow("image", img)
        cv2.waitKey(1)

        sleep(0.1) # to slow down the loop
        pass
except KeyboardInterrupt:
    pass










