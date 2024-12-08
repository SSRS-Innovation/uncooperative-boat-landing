import cv2
from dt_apriltags import Detector
import numpy as np
import matplotlib.pyplot as plt
import math
from geopy import distance
# import pyproj           # ungefär lika dåliga resultat med
# import pymap3d as pm    # dessa, pymap3d kanske har lite mer potential
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

# snabb 
old_video_string = ''
test_models = ['yolov5n6u.pt', 'yolov5nu.pt', 'yolov8n.pt', 'yolov8s.pt', 'yolov10n.pt', 'yolov9e.pt']
test_models = ['yolov5nu.pt','yolov8n.pt']
test_vids = {'0082': [[50*25, 75*25], [(2*60+45)*25, (3*60+2)*25], [(3*60+50)*25, (4*60+12)*25]],   # 0 verkar vara den enda bra i 0082       
             '0083': [[15*25,58*25]],
             '0084': [[1500,2400]], 
             '0085': [[(3*60+30)*25,(4*60+15)*25]]} # 0 här bra

for video_string in test_vids.keys():
    print(video_string)
    if video_string != '0083':  # Only to evaluate one videos clips
        continue
        
    for model_string in test_models: 
        y_model = YOLO(model_string)
        # y_model = RTDETR(model_string) 
        start_frame_number = test_vids[video_string][0][0]#1500 #60*4*25#1700 # 2000 är bra för 0085 # 1700 är bra för 0082 # 1000 för 0084 i rapporten
        end_frame = test_vids[video_string][0][1]#start_frame_number+900
        print(f"running video: {video_string}, frames: {start_frame_number}-{end_frame} number of frames: {end_frame-start_frame_number}")
        frame_rate = 25 # frame rate of video 
        frame_skips = 5 # every frame_skips frame gets sent to the algorithm

        april_tags = True
        visualization = False
        live_plotting = False

        # Just to trigger the screenshots at determined distances
        gps_distance_trigger1 = False
        gps_distance_trigger2 = False
        gps_distance_trigger3 = False

        detections_50 = 0 # the number of frames that has been processed by the network
        detections_100 = 0
        detections_g = 0
        multi_det_g = 0
        multi_det_100 = 0
        multi_det_50 = 0

        tot_frames_50 = 0
        tot_frames_100 = 0
        tot_frames_g = 0
        boat_detections = 0 # number of frames where a boat is detected by the network
        tot_frames = 0
        frame_times = []


        yolo_err_g = [] # yolo meassurement errors relative gps
        yolo_err_100 = []
        yolo_err_50 = []

        fyolo_err_g = [] # filtered yolo meassurement errors relative gps
        fyolo_err_100 = []
        fyolo_err_50 = []

        tag_err_g = [] # yolo meassurement errors relative april tags
        tag_err_100 = []
        tag_err_50 =  []

        f_tag_err_g = [] # filtered yolo meassurement errors relative april tags
        f_tag_err_100 = []
        f_tag_err_50 =  []


        yolo_errors = [] # meassured error relative boat gps
        fyolo_errors = [] # filtered error relative boat gps
        yolo_tag_errors = [] # meeassured error relative april tags
        fyolo_tag_errors = [] # filtered error relative april tags


        gps_distance_list = [] # gps distance between boat and drone
        cam_distance_list = [] # cam distance between boat and drone
        boat_yolo_pos_list = []
        boat_gps_pos_list = []
        apriltag_coords = []
        filtered_pos_list = []
        april_tags_pos_list = []
        a_tags_gps = []
        a_tags_yolo_list =[]

        bearing_window = 25 # bases the drone bearing on the angle between every bearing_window points
        drone_bearing = 0 # initial bearing before getting any values
        boat_dims = [14.2, 4.2]
        r_earth = 6378137 # m

        capture = cv2.VideoCapture("/mnt/c/plugg/Examensarbete/Videor/Daaataflyg1/DJI_"+video_string+".MP4")
        # capture = cv2.VideoCapture("/mnt/c/plugg/Examensarbete/Videor/Daaataflyg1/DJI_"+video_string+"LRF.MP4") # LRF
        # capture = cv2.VideoCapture("/mnt/c/plugg/Examensarbete/Videor/Daaataflyg1/DJI_"+video_string+"1080p.MP4") # 1080p

        file_path_camera_srt = '/mnt/c/plugg/Examensarbete/Videor/Daaataflyg1/DJI_'+video_string+'.SRT'
        file_path_phone =      '/mnt/c/plugg/Examensarbete/Videor/Daaataflyg1/gps_mob1.txt'

        Camera_matrix = [[1.31754161e+03, 0.00000000e+00, 1.01639924e+03],[0.00000000e+00, 1.31547107e+03, 5.24436193e+02],[0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]
        Camera_matrix = np.array(Camera_matrix)*2 # adjusting due to calibration being done in 1080p 1080p =*1, 4k = *2 ,720p = /1.5
        camera_params = (Camera_matrix[0,0], Camera_matrix[1,1], Camera_matrix[0,2], Camera_matrix[1,2])

        # kalman filter variables
        n = 4
        P = np.array(np.eye(n))     # [n x n] Prior covariance

        dt = (1/frame_rate)*frame_skips # delta t

        A = np.array([[1, 0, dt, 0], # state transition matrix
                    [0, 1, 0, dt],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

        xk = np.array([[57.672,11.841,0,0]]).T

        Q = np.array([[dt**3/3, 0, dt**2/2, 0],     # process noise matrix
                    [0, dt**3/3, 0, dt**2/2],
                    [dt**2/2, 0, dt, 0],
                    [0, dt**2/2, 0, dt]]) * 0.001   # process noise variance

        H = np.array([[1., 0, 0, 0],     # measurment matrix
                    [0, 1., 0, 0]])
        R = np.eye(2)*0.01               # meassurement noise


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


        frame_number = start_frame_number
        capture.set(cv2.CAP_PROP_POS_FRAMES, start_frame_number)
        while capture.isOpened():

            frame_number += 1
            if frame_number % 100 == 0 and live_plotting: # clear the figure due to performence loss
                plt.clf()
                print(drone_times[frame_number], boat_times[int(frame_number/25)]) 

            if frame_number > bearing_window:
                drone_bearing = math.atan2(d_long[frame_number]-d_long[frame_number-bearing_window], d_lat[frame_number]-d_lat[frame_number-bearing_window])

            if live_plotting:
                plot_point((d_long[frame_number],d_lat[frame_number]),drone_bearing,0.0001)

            ret, frame = capture.read()

            if not frame_number % frame_skips == 0: # skip frames to adjust frame rate
                continue

            # This is kind of cheating
            relative_bearing = math.atan2(b_interpolated_long[frame_number]-d_long[frame_number], b_interpolated_lat[frame_number]-d_lat[frame_number])

            if ret == True:
                gps_distance = distance.geodesic((d_lat[frame_number], d_long[frame_number]),(b_interpolated_lat[frame_number], b_interpolated_long[frame_number])).m # GPS distance between boat and drone
                tot_frames += 1
                if gps_distance > 100:
                    tot_frames_g += 1
                if 50 < gps_distance <= 100:
                    tot_frames_100 += 1
                if gps_distance <= 50:
                    tot_frames_50 += 1

                results = y_model(frame, stream=True, verbose=False, classes=8) # Verbose toggels outprint to console
                for result in results:
                    im_array = result.plot()
                frame_time = result.speed["preprocess"]+ result.speed["inference"]+ result.speed["postprocess"] # total time for processing of one frame
                frame_times.append(frame_time)

                if result.boxes and frame_number % frame_skips == 0:
                    gps_distance_list.append(gps_distance) # gps distances between boat and drone
                    boat_detections += 1
                    frame = im_array
                    bbox_center = int((result.boxes.xyxy[0][1]+result.boxes.xyxy[0][3])/2), int((result.boxes.xyxy[0][0]+result.boxes.xyxy[0][2])/2) # egentligen kan jag byta ordning på dessa så att det blir [y,x] men gör det i funktionen bbox_centerangle nu
                    bbox_cent_offset = bbox_centerangle(bbox_center, Camera_matrix)
                    effective_len = boat_angledim_compensator(boat_dims, bbox_cent_offset, b_bearing_interpolated[frame_number], drone_bearing, relative_bearing)

                    b_dist = boat_distance(effective_len, Camera_matrix, result.boxes) 
                    cam_distance_list.append(b_dist)

                    yolo_pos = distance.distance(meters=b_dist).destination((d_lat[frame_number],d_long[frame_number]),np.rad2deg(relative_bearing)+bbox_cent_offset) # egentligen postionen inte distance
                    yk = np.array([yolo_pos.latitude, yolo_pos.longitude, 0,0]).transpose() # är denna helt korrekt?
                    xk, P = kalman.kalman_filter(yk, xk, P, A, Q, H, R)
                    
                    yolo_err = distance.geodesic((yolo_pos.latitude, yolo_pos.longitude), 
                                                (b_interpolated_lat[frame_number], b_interpolated_long[frame_number])).m

                    yolo_errors.append(yolo_err)
                    fyolo_error = distance.geodesic((xk[0,0], xk[1,1]), 
                                                (b_interpolated_lat[frame_number], b_interpolated_long[frame_number])).m
                    fyolo_errors.append(fyolo_error)
                    boat_yolo_pos_list.append([yolo_pos.longitude, yolo_pos.latitude]) # long lat from yolo
                    boat_gps_pos_list.append([b_interpolated_long[frame_number], b_interpolated_lat[frame_number]])
                    filtered_pos_list.append([xk[1,1], xk[0,0]])
                    

                    if gps_distance > 100:# and 3000 > bbox_center[1] > 800 and 1800 > bbox_center[0] > 300: # ???and bara_en_bounding box??? : # något sådant här?
                        detections_g += 1
                        yolo_err_g.append(yolo_err)
                        fyolo_err_g.append(fyolo_error)
                        if len(result.boxes.xyxy) > 1:
                            multi_det_g += 1

                    if 50 < gps_distance <= 100:# and 3000 > bbox_center[1] > 800 and 1800 > bbox_center[0] > 300: # ???and bara_en_bounding box??? : # något sådant här?
                        detections_100 += 1
                        yolo_err_100.append(yolo_err)
                        fyolo_err_100.append(fyolo_error)
                        if len(result.boxes.xyxy) > 1: 
                            multi_det_100 += 1

                    if gps_distance <= 50:# and 3000 > bbox_center[1] > 800 and 1800 > bbox_center[0] > 300: # ???and bara_en_bounding box??? : # något sådant här?
                        detections_50 += 1
                        yolo_err_50.append(yolo_err)
                        fyolo_err_50.append(fyolo_error)
                        if len(result.boxes.xyxy) > 1: 
                            multi_det_50 += 1

                    if live_plotting:
                        plt.scatter(yolo_pos.longitude, yolo_pos.latitude ,color='purple', label='Approx pos') # approx position  
                        plt.scatter(xk[1,1], xk[0,0], color='orange', label='filtered pos') # filtered position

                if april_tags:
                    only_once = True # makes sure that we only compare GPS distance to one of the arpil tags
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    tags = at_detector.detect(frame, True, camera_params, 0.1735) #  130 mm är våran man mäter insidan och den anges i meter https://github.com/AprilRobotics/apriltag?tab=readme-ov-file#pose-estimation (blir 3 pixlar man mäter)
                    # print(tags) # https://github.com/duckietown/lib-dt-apriltags/tree/daffy
                                # pose_t är på formen X,Y,Z och med högerhandkoordinatsystem blir det: [högerled_från kameran, höjdled, avstånd pekar ut ur kameran]
                    for tag in tags:
                        dx = tag.pose_t[0] # X coordinate 
                        dy = tag.pose_t[2] #  Z coordinate boats relative position to the camera (no altitude yet)
                        bca = boat_cam_angle(drone_bearing, tag, Camera_matrix)
                        approx_drone_bearing = np.rad2deg(drone_bearing) + bca#tag_center_angle(tags[0])

                        # m = (1 / ((2 * math.pi / 360) * r_earth/1000)) / 1000
                        
                        newlatlong = distance.distance(meters=tag.pose_t[2][0]).destination((d_lat[frame_number],d_long[frame_number]),approx_drone_bearing)
                        new_latitude, new_longitude = newlatlong.latitude, newlatlong.longitude 
                        # plt.scatter(new_longitude,new_latitude,color='green', label='April tags')#, label='April tag + drone gps') # och denna 
                        apriltag_coords.append([new_longitude,new_latitude])
                        if only_once:
                            tag_error = distance.geodesic((yolo_pos.latitude, yolo_pos.longitude), 
                                                (new_latitude, new_longitude)).m
                            fyolo_tag_error = distance.geodesic((xk[0,0],xk[1,1]),            
                                                                (new_latitude, new_longitude)).m
                            if gps_distance > 100:# and 3000 > bbox_center[1] > 800 and 1800 > bbox_center[0] > 300: # ???and bara_en_bounding box??? : # något sådant här?
                                tag_err_g.append(tag_error)
                                f_tag_err_g.append(fyolo_tag_error)

                            elif 50 < gps_distance <= 100:# and 3000 > bbox_center[1] > 800 and 1800 > bbox_center[0] > 300: # ???and bara_en_bounding box??? : # något sådant här?
                                tag_err_100.append(tag_error)
                                f_tag_err_100.append(fyolo_tag_error)

                            elif gps_distance <= 50:# and 3000 > bbox_center[1] > 800 and 1800 > bbox_center[0] > 300: # ???and bara_en_bounding box??? : # något sådant här?
                                tag_err_50.append(tag_error)
                                f_tag_err_50.append(fyolo_tag_error)
                            else:
                                print("somehting is wrong with the distances")

                        
                            fyolo_tag_errors.append(fyolo_tag_error)
                            yolo_tag_errors.append(tag_error)
                            
                            
                            april_tags_pos_list.append([new_longitude, new_longitude])

                            # a_tags_gps.append([b_interpolated_long[frame_number], b_interpolated_lat[frame_number]])
                            a_tags_yolo_list.append([yolo_pos.longitude, yolo_pos.latitude]) # yolo positions
                            only_once = False

                        for idx in range(len(tag.corners)):
                            cv2.line(frame, tuple(tag.corners[idx-1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))

                        cv2.putText(frame, str(tag.tag_id),
                            org=(tag.corners[0, 0].astype(int)+10,tag.corners[0, 1].astype(int)+10),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=0.8,
                            color=(0, 0, 255))


                if visualization:
                    cv2.imshow('Detected tags', frame)

                    inference_time = result.speed['inference']
                    name_string = f'S1_17_model-{model_string}_vid-{video_string}_frame-{frame_number}_dist-{gps_distance}_speed-{inference_time}'
                    # print(name_string)

                    # if gps_distance < 50.5 and not gps_distance_trigger1:
                    #     # Save image
                    #     cv2.imwrite(f'/mnt/c/plugg/Examensarbete/Bilder/{name_string}.png', frame)
                    #     gps_distance_trigger1 = True


                    # if gps_distance < 120.5 and not gps_distance_trigger2:
                    #     # Save image
                    #     cv2.imwrite(f'/mnt/c/plugg/Examensarbete/Bilder/{name_string}.png', frame)
                    #     gps_distance_trigger2 = True

                    # if gps_distance < 20 and not gps_distance_trigger3:
                    #     # Save image
                    #     cv2.imwrite(f'/mnt/c/plugg/Examensarbete/Bilder/{name_string}.png', frame)
                    #     gps_distance_trigger3 = True
                    if live_plotting:
                        plt.scatter(d_long[frame_number],d_lat[frame_number],color='red' , label='drone gps') # 
                        plt.scatter(b_interpolated_long[frame_number],b_interpolated_lat[frame_number],color='blue',label='boat gps') # boats coordinate
                        # plt.legend(['drone bearing', 'approx position','drone gps', 'boat gps'],loc='upper right')
                        plt.draw()
                        plt.pause(0.00001)
                        
                if cv2.waitKey(1) & 0xFF == ord('q'): 
                    break
                elif frame_number == end_frame:
                    break
            if frame_number % 100 == 0:
                print(f"frame number: {frame_number}, est time left: {((end_frame-frame_number)*(sum(frame_times)/len(frame_times)))/1000}")

        # # Calculte the errors dependning on the distance aswell as splitting it up to the three categories and see mean errors.
        inital_skip = 0
        npgps = np.array(boat_gps_pos_list) # gps positions long lat
        npcam = np.array(boat_yolo_pos_list) # cam meassurments
        gps_distance_array = np.array(gps_distance_list)
        npfiltered = np.array(filtered_pos_list)
        position_error_filtered = []
        position_error_meassurements = []
        diff_apriltags = []
        for i in range(inital_skip,len(npgps)): # skipping points for kalman filter i think?
            position_error_filtered.append(distance.geodesic((npgps[i,1],npgps[i,0]),(npfiltered[i,1],npfiltered[i,0])).m)
            position_error_meassurements.append(distance.geodesic((npgps[i,1],npgps[i,0]),(npcam[i,1],npcam[i,0])).m)

        april_tags_diff = []
        april_tags_pos_array = np.array(april_tags_pos_list)
        a_tags_yolo_array = np.array(a_tags_yolo_list)
        for i in range(inital_skip, len(a_tags_yolo_list)): 
            april_tags_diff.append(distance.geodesic((a_tags_yolo_array[i,1],a_tags_yolo_array[i,0]),(april_tags_pos_array[i,1],april_tags_pos_array[i,0])).m)
            
            
        # borde bara kunna göra detta med listor o grejer typ gps_distance_list < 50 etc och sen nparray av listorna och ta felen
        # måste indexera så att det jag bara plockar de avtånden där det blivit detektioner
        err50 = np.mean(np.array(position_error_meassurements)[gps_distance_array<=50])
        err100 = np.mean(np.array(position_error_meassurements)[(100>=gps_distance_array) & (gps_distance_array>50)])
        errg = np.mean(np.array(position_error_meassurements)[gps_distance_array>100])

        ferr50 = np.mean(np.array(position_error_filtered)[gps_distance_array<=50])
        ferr100 = np.mean(np.array(position_error_filtered)[(100>=gps_distance_array) & (gps_distance_array>50)])
        ferrg = np.mean(np.array(position_error_filtered)[gps_distance_array>100])

        if torch.cuda.is_available():
            cam_distance_list = list(map(lambda x: x.item(), cam_distance_list))
            # print(np.array(cam_distance_list))

        distance_errors = np.array(cam_distance_list)-gps_distance_array # only distance error not position errors


        # plt.close()
        plt.figure()
        plt.title(f'Model: {model_string}, Video:{video_string}, frames: {start_frame_number} - {end_frame}')
        plt.scatter(gps_distance_array, position_error_meassurements)
        plt.xlabel('GPS distance [M]')
        plt.ylabel('Position error [M]')


        plt.figure()
        plt.title(f'Model: {model_string}, Video:{video_string}, frames: {start_frame_number} - {end_frame}')
        plt.scatter(npgps[:,0], npgps[:,1] ,color='blue' , label='boat gps')
        plt.scatter(npcam[:,0], npcam[:,1], color='purple', label='camera meassurements')
        plt.scatter(npfiltered[:,0], npfiltered[:,1], color='orange', label='filtered position')
        if april_tags:
            npac = np.array(apriltag_coords)
            plt.scatter(npac[:,0],npac[:,1], color='green', label='april tags')
        plt.legend()

        plt.figure()
        plt.title(f'Model: {model_string}, Video:{video_string}, frames: {start_frame_number} - {end_frame}')
        plt.scatter(gps_distance_array, distance_errors)
        plt.xlabel('GPS distance [M]')
        plt.ylabel('distance error [M]')


        # plt.figure()
        # plt.title(f'Model: {model_string}, Video:{video_string}, frames: {start_frame_number} - {end_frame}')



        print(f'model: {model_string}, start frame: {start_frame_number}, end frame: {end_frame}, vid: {video_string}')
        print(f'GPS filtered position error: {np.mean(position_error_filtered[inital_skip:])}')
        print(f'GPS measured position error: {np.mean(position_error_meassurements[inital_skip:])}')
        if len(yolo_err_g) == 0:
            yolo_err_g.append(-1)
            fyolo_err_g.append(-1)

        for l in [f_tag_err_50, f_tag_err_100, f_tag_err_g, tag_err_50, tag_err_100, tag_err_g]:
            if len(l) == 0:
                l.append(-1)
        print(f'\nACTUAL yolo ERROR: {sum(yolo_err_50)/len(yolo_err_50)} 50m, {sum(yolo_err_100)/len(yolo_err_100)} 100m, {sum(yolo_err_g)/len(yolo_err_g)} >100m')
        print(f'ACTUAL filtered yolo ERROR: {sum(fyolo_err_50)/len(fyolo_err_50)} 50m, {sum(fyolo_err_100)/len(fyolo_err_100)} 100m, {sum(fyolo_err_g)/len(fyolo_err_g)} >100m')
        if april_tags:
            print(f'tag errors {sum(tag_err_50)/len(tag_err_50)} 50m, {sum(tag_err_100)/len(tag_err_100)} 100m, {sum(tag_err_g)/len(tag_err_g)} >100m')
            print(f'filtered tag errors {sum(f_tag_err_50)/len(f_tag_err_50)} 50m, {sum(f_tag_err_100)/len(f_tag_err_100)} 100m, {sum(f_tag_err_g)/len(f_tag_err_g)} >100m\n')

        print(f'ALL FOR GPS: err50: {err50}, err100: {err100}, errg: {errg}, toterr: {np.mean([err50,err100,errg])}')
        print(f'detections_50: {detections_50}, detections_100: {detections_100}, detections_g: {detections_g}')
        print(f'tot_frames_50: {tot_frames_50}, tot_frames_100: {tot_frames_100}, tot_frames_g: {tot_frames_g}')

        # print(f'%det50%: {detections_50/tot_frames_50}, %det100%: {detections_100/tot_frames_100}, %detg%: {detections_g/tot_frames_g}')
        print(f'multi_det_50: {multi_det_50}, multi_det_100: {multi_det_100}, multi_det_g: {multi_det_g}')
        det_count_tot = detections_g+detections_100+detections_50
        print(f'total_frame_count: {tot_frames}, Detected frames: {det_count_tot}, det%: {det_count_tot/tot_frames}')
        print(f'Mean speed ms on laptop: {sum(frame_times)/tot_frames}') # inte pålitligt sätt 
        if april_tags:
            print(f'april tags error: {sum(april_tags_diff)/len(a_tags_yolo_list)}')

        if video_string != old_video_string:
            old_video_string = video_string
            with open('output.txt', 'a') as file:
                            file.write(f"""\\begin{{table}}[h!] \centering \caption{{ A table of video:{video_string}, frames: {start_frame_number} - {end_frame}}} \\begin{{tabular}}{{|c|c|c|c|c|c|c|c|c|}} 
\hline \multicolumn{{9}}{{|c|}}{{Slow boat (5-7 knots) {frame_rate/frame_skips:.1f} fps}} \\\\
\hline Model & d [M]& Det [N] & Mdet [N] &$\epsilon_{{'p'}}$ [M] & $\epsilon_{{fp}}$ [M] & $\epsilon_{{at}}$ [M] & $\epsilon_{{fat}}$ [M] & t [ms]\\\\""")


        with open('output.txt', 'a') as file:
            file.write(f"""\n\hline 
    \multirow{'{3}'}{'{*}'}{{{model_string}}}
    & <50: & {detections_50} & {multi_det_50} & {sum(yolo_err_50)/len(yolo_err_50):.2f} & {sum(fyolo_err_50)/len(fyolo_err_50):.2f} & {sum(tag_err_50)/len(tag_err_50):.2f} & {sum(f_tag_err_50)/len(f_tag_err_50):.2f} &  \\\\
    & 50-100: & {detections_100} & {multi_det_100} & {sum(yolo_err_100)/len(yolo_err_100):.2f} & {sum(fyolo_err_100)/len(fyolo_err_100):.2f} & {sum(tag_err_100)/len(tag_err_100):.2f} & {sum(f_tag_err_100)/len(f_tag_err_100):.2f} & {sum(frame_times)/tot_frames:.2f} \\\\
    & >100: & {detections_g} & {multi_det_g} & {sum(yolo_err_g)/len(yolo_err_g):.2f} & {sum(fyolo_err_g)/len(fyolo_err_g):.2f} & {sum(tag_err_g)/len(tag_err_g):.2f} & {sum(f_tag_err_g)/len(f_tag_err_g):.2f} &  \\\\""")

    with open('output.txt','a') as file:
        file.write('''\n \hline
\end{tabular} 
\end{table}''')


# plt.figure(3)
# np_apos = np.array(april_tags_pos_list)
# np_aypos = np.array(a_tags_yolo_list)
# plt.scatter(np_apos[:,0],np_apos[:,1], color='green', label='april tags')
# plt.scatter(np_aypos[:,0],np_aypos[:,1], color='purple', label='yolo meassurements')
# plt.legend()
plt.show()













