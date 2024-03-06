import cv2
from dt_apriltags import Detector
import numpy as np
import matplotlib.pyplot as plt
import math
from geopy import distance
import pyproj           # ungefär lika dåliga resultat med
import pymap3d as pm    # dessa, pymap3d kanske har lite mer potential
from ultralytics import YOLO
from ultralytics import RTDETR
import torch
import kalman
import files_process
from  boat_positioning import bbox_centerangle, boat_angledim_compensator, boat_cam_angle, boat_distance

print(torch.cuda.is_available())

def plot_point(point, angle, length):
     # unpack the first point
     x, y = point

     # find the end point
     endy = abs(y + length * math.sin(angle))
     endx = abs(x + length * math.cos(angle))
     plt.plot([x, endx], [y, endy])
     plt.draw()



y_model = YOLO('yolov8n.pt')
# y_model = YOLO('yolov9.pt')
# y_model = YOLO('yolov6n.yaml') Verkar inte kunna upptäcka båten särskilt bra
# y_model = RTDETR('rtdetr-l.pt')

frame_skips = 12 # out of how many frames we feed to the algorithm, this means that every nth fram is fed to the algorithm
april_tags = False
visualization = True
boat_dims = [14.2, 4.2]
r_earth = 6378137 # m

video_string = '0082' # 
capture = cv2.VideoCapture("/mnt/c/plugg/Examensarbete/Videor/Daaataflyg1/DJI_"+video_string+".MP4")
# capture = cv2.VideoCapture("/mnt/c/plugg/Examensarbete/Videor/Daaataflyg1/DJI_"+video_string+"LRF.MP4") # LRF
# capture = cv2.VideoCapture("/mnt/c/plugg/Examensarbete/Videor/Daaataflyg1/DJI_"+video_string+"1080p.MP4") # 1080p

# capture = cv2.VideoCapture("/mnt/c/plugg/Examensarbete/Videor/DJI_kalibrering/DJI_0089_3.087_utv_mtb_297.MP4") # utvärderings captures:  DJI_0087_11.102_utv, DJI_0088_7.390_utv, DJI_0089_3.087_utv_mtb_297
file_path_camera_srt = '/mnt/c/plugg/Examensarbete/Videor/Daaataflyg1/DJI_'+video_string+'.SRT'
file_path_phone =      '/mnt/c/plugg/Examensarbete/Videor/Daaataflyg1/gps_mob1.txt'

Camera_matrix = [[1.31754161e+03, 0.00000000e+00, 1.01639924e+03],[0.00000000e+00, 1.31547107e+03, 5.24436193e+02],[0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]
Camera_matrix = np.array(Camera_matrix)*2 # adjusting due to calibration being done in 1080p 1080p =*1, 4k = *2 ,720p = /1.5
camera_params = (Camera_matrix[0,0], Camera_matrix[1,1], Camera_matrix[0,2], Camera_matrix[1,2])

# kalman filter variables
n = 4
m = 2 
dt = 0.040*frame_skips # delta t
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


drone_times, d_lat, d_long, rel_alt =  files_process.read_dji_srt(file_path_camera_srt)
boat_times, b_lat, b_long, b_alt, b_bearing = files_process.read_phone_file(file_path_phone)
boat_times, b_lat, b_long = files_process.time_sync(boat_times, drone_times, b_lat, b_long)
b_interpolated_lat, b_interpolated_long, b_alt_interpolated, b_bearing_interpolated = files_process.interpolate_points(b_lat, b_long, b_alt, b_bearing)



if (capture.isOpened()== False):  
    print("could not open video file")

at_detector = Detector(families='tag36h11',
                       nthreads=12,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)


start_frame_number = 1000 #60*4*25#1700 # 2000 är bra för 0085 # 1700 är bra för 0082
frame_number = start_frame_number
bearing_window = 25
drone_bearing = 0 # initial bearing before getting any values
capture.set(cv2.CAP_PROP_POS_FRAMES, start_frame_number)
while capture.isOpened():

    frame_number += 1
    if frame_number % 300 == 0: # clear the figure every 50 frames due to performence loss
        plt.clf()
        print(drone_times[frame_number], boat_times[int(frame_number/25)]) 


    # if frame_number > bearing_window:
    if frame_number > 2:
        # drone_bearing = math.atan2(d_long[frame_number]-d_long[frame_number-bearing_window], d_lat[frame_number]-d_lat[frame_number-bearing_window])
        drone_bearing = math.atan2(b_interpolated_long[frame_number]-d_long[frame_number], b_interpolated_lat[frame_number]-d_lat[frame_number])

    plot_point((d_long[frame_number],d_lat[frame_number]),drone_bearing,0.0001)


    ret, frame = capture.read()
    if not frame_number % frame_skips == 0: # skip frames to adjust frame rate
        continue

    # This is cheating
    relative_bearing = math.atan2(b_interpolated_long[frame_number]-d_long[frame_number], b_interpolated_lat[frame_number]-d_lat[frame_number])


    if ret == True:

        results = y_model(frame, stream=True, verbose=False) # Verbose toggels outprint to console
        for result in results:
            im_array = result.plot()
        if result.boxes and frame_number % frame_skips == 0:
            frame = im_array
            bbox_center = int((result.boxes.xyxy[0][1]+result.boxes.xyxy[0][3])/2), int((result.boxes.xyxy[0][0]+result.boxes.xyxy[0][2])/2) # egentligen kan jag byta ordning på dessa så att det blir [y,x] men gör det i funktionen bbox_centerangle nu
            bbox_cent_offset = bbox_centerangle(bbox_center, Camera_matrix)
            effective_len = boat_angledim_compensator(boat_dims, bbox_cent_offset, b_bearing_interpolated[frame_number], drone_bearing, relative_bearing)

            b_dist = boat_distance(effective_len, Camera_matrix, result.boxes) 

            yolo_dist = distance.distance(meters=b_dist).destination((d_lat[frame_number],d_long[frame_number]),np.rad2deg(drone_bearing)+bbox_cent_offset)
            # yolo_dist = distance.distance(meters=b_dist).destination((d_lat[frame_number],d_long[frame_number]),relative_bearing)

            yolo_lat, yolo_long = yolo_dist.latitude, yolo_dist.longitude

            yk = np.array([yolo_lat,yolo_long , 0,0]).transpose()
            xk, P = kalman.kalman_filter(yk, xk, P, A, Q, H, R)
            
            plt.scatter(yolo_long, yolo_lat ,color='purple', label='Approx pos') # approx position  
            plt.scatter(xk[1,1], xk[0,0], color='orange', label='filtered pos') # filtered position

        if april_tags:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            tags = at_detector.detect(frame, True, camera_params, 0.1735) #  130 mm är våran man mäter insidan och den anges i meter https://github.com/AprilRobotics/apriltag?tab=readme-ov-file#pose-estimation (blir 3 pixlar man mäter)
            # print(tags) # https://github.com/duckietown/lib-dt-apriltags/tree/daffy
                        # pose_t är på formen X,Y,Z och med högerhandkoordinatsystem blir det: [högerled_från kameran, höjdled, avstånd pekar ut ur kameran]
            for tag in tags:
                dx = tag.pose_t[0] # X coordinate 
                dy = tag.pose_t[2] # Z coordinate boats relative position to the camera (no altitude yet)
                bca = boat_cam_angle(drone_bearing, tag, Camera_matrix)
                approx_drone_bearing = np.rad2deg(drone_bearing) + bca#tag_center_angle(tags[0])

                m = (1 / ((2 * math.pi / 360) * r_earth/1000)) / 1000
                
                newlatlong = distance.distance(meters=tag.pose_t[2][0]).destination((d_lat[frame_number],d_long[frame_number]),approx_drone_bearing)
                new_latitude, new_longitude = newlatlong.latitude, newlatlong.longitude
                plt.scatter(new_longitude,new_latitude,color='green', label='April tags')#, label='April tag + drone gps') # och denna 

                for idx in range(len(tag.corners)):
                    cv2.line(frame, tuple(tag.corners[idx-1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))

                cv2.putText(frame, str(tag.tag_id),
                    org=(tag.corners[0, 0].astype(int)+10,tag.corners[0, 1].astype(int)+10),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=0.8,
                    color=(0, 0, 255))
                
        if visualization:
            cv2.imshow('Detected tags', frame)
            # k = cv2.waitKey(0)
            # if tags:
            plt.scatter(d_long[frame_number],d_lat[frame_number],color='red' , label='drone gps') # 
            plt.scatter(b_interpolated_long[frame_number],b_interpolated_lat[frame_number],color='blue',label='boat gps') # boats coordinate
            # plt.legend(['drone bearing', 'approx position','drone gps', 'boat gps'],loc='upper right')
            plt.draw()
            plt.pause(0.00001)
            
        if cv2.waitKey(1) & 0xFF == ord('q'): 
            break














