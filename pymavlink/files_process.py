import numpy as np
from datetime import datetime, timedelta

def read_dji_srt(file_path_camera_srt):
    drone_times = []
    d_lat = []
    d_long = []
    rel_alt = [] 
    with open(file_path_camera_srt) as file:
        format_string = '%Y-%m-%d %H:%M:%S.%f'
        for line in file:
            if line.startswith('2024-'):
                datetime_object = datetime.strptime(line.rstrip(), format_string) # rstrip to remove trailing newline
                drone_times.append(datetime_object) 

            if line.startswith('['):
                d_lat.append(float(line.split('latitude: ')[1][:8]))
                d_long.append(float(line.split('longitude: ')[1][:8]))
                rel_alt.append(float(line.split('rel_alt: ')[1][:4])) 
    return drone_times, d_lat, d_long, rel_alt
            
def read_phone_file(file_path_phone):
    boat_times = []
    b_lat = []
    b_long = []
    b_alt = []
    b_bearing =[]
    with open(file_path_phone) as file_p:
        format_string = '%Y-%m-%d %H:%M:%S'
        for line in file_p:
            splitted = line.split(',')
            if splitted[2] == 'latitude': # Skipping first entry in the file since it is a string of what it will contain 
                continue
            boat_times.append(datetime.strptime(splitted[1], format_string) + timedelta(hours=1)) 
            b_lat.append(float(splitted[2]))
            b_long.append(float(splitted[3]))
            b_alt.append(float(splitted[5]))
            b_bearing.append(float(splitted[8])) # added start value in file on second row was empty otherwise
    return boat_times, b_lat, b_long, b_alt, b_bearing



def time_sync(boat_times, drone_times, b_lat, b_long):
    bools = np.logical_and(np.array(boat_times) >= np.array(drone_times[0]), np.array(boat_times) <= np.array(drone_times[-1])) # greater or equal then the first element and less or equal then last element
    b_lat = np.array(b_lat)[bools]
    b_long = np.array(b_long)[bools] 
    boat_times = np.array(boat_times)[bools]
    return boat_times, b_lat, b_long

def interpolate_points(b_lat, b_long, b_alt, b_bearing):
# interpolate the mobile gps data so it gets as many points as there are frames(as the mavic drone has also)
# we will assume constand speed between the data points
    b_interpolated_lat = []
    b_interpolated_long = []
    b_alt_interpolated = []
    b_bearing_interpolated = []
    # boat_times_interpolated =[]
    num_interpolations = 25
    for i in range(1, len(b_lat)):

        b_interpolated_lat.append(np.linspace(b_lat[i-1],b_lat[i],num_interpolations))
        b_interpolated_long.append(np.linspace(b_long[i-1],b_long[i],num_interpolations))
        b_alt_interpolated.append(np.linspace(b_alt[i-1],b_alt[i],num_interpolations))
        b_bearing_interpolated.append(np.linspace(b_bearing[i-1],b_bearing[i],num_interpolations))


    b_interpolated_lat = np.concatenate(b_interpolated_lat, axis=0)
    b_interpolated_long = np.concatenate(b_interpolated_long, axis=0)
    b_alt_interpolated = np.concatenate(b_alt_interpolated, axis=0)
    b_bearing_interpolated = np.concatenate(b_bearing_interpolated, axis=0)
    return b_interpolated_lat, b_interpolated_long, b_alt_interpolated, b_bearing_interpolated

