import numpy as np
import math

def boat_cam_angle(boat_bearing, tag, camera_matrix):
    x0, y0 = [camera_matrix[0,2],camera_matrix[1,2]] # should be [x0, y0] camera center
    # tag_pixel_offset = tag.center - [x0, y0]
    invcam = np.linalg.inv(camera_matrix)
    l_center = invcam.dot([x0, y0, 1.0])
    extended = np.append(tag.center, 1 )
    l_tag = invcam.dot(extended) # behöva concantenata denna??
    cos_angle = l_center.dot(l_tag) / (np.linalg.norm(l_center) * np.linalg.norm(l_tag))
    angle_radians = np.arccos(cos_angle)
    tag_deg_offset = np.rad2deg(angle_radians)
    # approx_drone_bearing = boat_bearing + tag_deg_offset
    return tag_deg_offset # rad2deg korrekt här?

def bbox_centerangle(bbox_center, camera_matrix): # function that takes the angle from the center of the screen to the center of the bounding box
    x0, y0 = [camera_matrix[0,2],camera_matrix[1,2]]
    invcam = np.linalg.inv(camera_matrix)
    l_center = invcam.dot([x0, y0, 1.0])
    xb, yb = bbox_center
    # print([yb, xb, 1])
    l_bbox = invcam.dot([yb, xb, 1])
    cos_angle = l_center.dot(l_bbox) / (np.linalg.norm(l_center) * np.linalg.norm(l_bbox))
    angle_radians = np.arccos(cos_angle)
    bbox_deg_offset = np.rad2deg(angle_radians)
    return bbox_deg_offset




# this is not used now, this works ok if we use the rel_bearing
def boat_angledim_compensator(boat_dims, boat_pic_angle, boat_bearing, drone_bearing): #, boat_distance, bbox_centerangle):
    # compensate for when the boat is turned in different ways towards the camera for example if its 90degrees bounding box should be length meters instead of width meters.
    # fix so if its not straight infront of the camera we have to compensate for that aswell, 
    # also maybe take into account that the part of the boat that is furthest from the camera is deeper in the picture then the distance

    drone_bearingdeg = np.rad2deg(drone_bearing)
    drone_bearingdeg = drone_bearingdeg%360
    v = drone_bearingdeg - boat_bearing + boat_pic_angle # mod 360 fixes - negative degrees to positive

    # print(drone_bearingdeg, boat_bearing, boat_pic_angle)
    # print(v)

    # cheating we dont have relative bearing # but we are not using it anyways
    # v = rel_bearing


    b_length, b_width = boat_dims
    side_length = abs(b_width*math.cos(np.deg2rad(v)))+abs(b_length*math.sin(np.deg2rad(v)))# this length is not taking the perspective distorsions into account
    # print(side_length)
    return side_length

def boat_distance(effective_len, camera_matrix, bbox):
    compen_len = effective_len # 4.2 # 
    
    focal_l = camera_matrix[0][0]
    dist = focal_l*compen_len/(bbox.xyxy[0][2]-bbox.xyxy[0][0])  
    return dist



def boat_distance_gazebo(effective_len, camera_matrix, max_corner, min_corner):
    compen_len = effective_len #4.2 # 
    
    focal_l = camera_matrix[0][0] 
    dist = focal_l*compen_len/(max_corner[0]-min_corner[0])  
    return dist
