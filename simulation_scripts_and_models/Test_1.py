def tester():
    from pymavlink import mavutil
    # from pymavlink import mavwp
    # from multiprocessing import Process



    # import os
    from subprocess import Popen, PIPE
    from time import sleep
    import math
    # from positioner_gazebo_box import positioner

    def takeoff(the_connection):
        print("takeoff init")
        the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,10,0,0,math.nan,0,0,10)

    flightstage = 0
    boatstage = 0

    command_list_drone = "sim_vehicle.py -v ArduPlane -f gazebo-zephyr --model JSON --map --console --out 0.0.0.0:14551 --out 0.0.0.0:14552 -i0 -L Kattegatt".split()
    command_list_boat = "sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out 0.0.0.0:14561 --out 0.0.0.0:14562 -i1 -L Kattegatt".split()
    gazebo_sim = "gz sim island.sdf -r".split()

    Popen(gazebo_sim)
    sleep(1)
    drone_process = Popen(command_list_drone, stdin=PIPE)
    sleep(1)
    boat_process = Popen(command_list_boat, stdin=PIPE)

    drone_connection = mavutil.mavlink_connection('udpin:localhost:14551')
    boat_connection = mavutil.mavlink_connection('udpin:localhost:14561') 

    print("Waiting for heartbeat from drone")
    drone_connection.wait_heartbeat()
    print("waiting for heartbeat from boat drone")
    boat_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (drone_connection.target_system, drone_connection.target_component))
    print("heartbeats recieved in TEST")

    def set_parameter_drone(param_id, param_value, param_type):
    # Create the MAVLink parameter set message
        drone_connection.mav.param_set_send(
        drone_connection.target_system,
        drone_connection.target_component,
        param_id.encode('utf-8'),
        param_value,
        param_type
        )
        ack = drone_connection.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
        print(f'Successfully set {param_id} to {param_value}')

    set_parameter_drone('SCR_ENABLE', 1, mavutil.mavlink.MAV_PARAM_TYPE_UINT8)

    #reboot
    drone_connection.mav.command_long_send(boat_connection.target_system, boat_connection.target_component, mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN ,0,1,0,0,0,0,0,0)
    print("rebooting")
    sleep(20)
    set_parameter_drone('SHIP_ENABLE', 1, mavutil.mavlink.MAV_PARAM_TYPE_UINT8)
    set_parameter_drone('SERVO3_FUNCTION', 0 , mavutil.mavlink.MAV_PARAM_TYPE_UINT8)

    # -35.36202481 149.16409768
    latitude = 11.294003
    longitude = 57.602939
    altitude = 20  # Altitude in meters 
    # latitude = -35.36202481
    # longitude = 149.16409768
    # altitude = 20  # Altitude in meters

    # Create the MISSION_ITEM_INT message
    msg = boat_connection.mav.mission_item_int_encode(
        boat_connection.target_system,  # Target system ID (0 for broadcast)
        boat_connection.target_component,  # Target component ID (0 for broadcast)
        0,  # Sequence number (1 for the first waypoint)
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # Frame of reference
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  # Command ID for waypoint
        2,  # Current (set to 0 for new waypoints)
        1,  # Autocontinue (set to 0 for new waypoints)
        0,  # Param 1 (not used for waypoints)
        0,  # Param 2 (not used for waypoints)
        0,  # Param 3 (not used for waypoints)
        0,  # Param 4 (not used for waypoints)
        int(latitude * 1e7),  # Param 5 (latitude in degrees * 1e7)
        int(longitude * 1e7),  # Param 6 (longitude in degrees * 1e7)
        int(altitude),  # Param 7 (altitude in meters)
        0
    )

    msg2 = boat_connection.mav.mission_item_int_encode(
        boat_connection.target_system,  # Target system ID (0 for broadcast)
        boat_connection.target_component,  # Target component ID (0 for broadcast)
        0,  # Sequence number (1 for the first waypoint)
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # Frame of reference
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  # Command ID for waypoint
        2,  # Current (set to 0 for new waypoints)
        1,  # Autocontinue (set to 0 for new waypoints)
        0,  # Param 1 (not used for waypoints)
        0,  # Param 2 (not used for waypoints)
        0,  # Param 3 (not used for waypoints)
        0,  # Param 4 (not used for waypoints)
        int(latitude * 1e7),  # Param 5 (latitude in degrees * 1e7)
        int(longitude * 1e7),  # Param 6 (longitude in degrees * 1e7)
        int(-1),  # Param 7 (altitude in meters)
        0
    )

    msg3 = boat_connection.mav.mission_item_int_encode(
        boat_connection.target_system,  # Target system ID (0 for broadcast)
        boat_connection.target_component,  # Target component ID (0 for broadcast)
        0,  # Sequence number (1 for the first waypoint)
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # Frame of reference
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  # Command ID for waypoint
        2,  # Current (set to 0 for new waypoints)
        1,  # Autocontinue (set to 0 for new waypoints)
        0,  # Param 1 (not used for waypoints)
        0,  # Param 2 (not used for waypoints)
        0,  # Param 3 (not used for waypoints)
        0,  # Param 4 (not used for waypoints)
        int((latitude-10) * 1e7),  # Param 5 (latitude in degrees * 1e7)
        int(longitude * 1e7),  # Param 6 (longitude in degrees * 1e7)
        int(-1),  # Param 7 (altitude in meters)
        0
    )


    # print("before sending messages ==================")
    # drone_connection.mav.send(scr_enable_msg)
    # sleep(1)
    # drone_connection.mav.send(param_ftp)
    # sleep(1)
    # drone_connection.mav.send(scr_heap_size_msg)
    # print("after sending messages ")
    # reboot
    # drone_connection.mav.send(ship_enable_msg)




    # param set scr_enable 1                        /Enable scripting on QuadPlane
    # param ftp                                      /Refresh params to see SCR_ params
    # param set scr_heap_size 100000                 /Set memory large enough to run script
    # reboot 
    # param set ship_enable 1






    # print(dir(boat_connection))
    # print(dir(drone_connection))
    boat_connection.mav.command_long_send(boat_connection.target_system, boat_connection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,0,1,4,0,0,0,0,0)# 13 is takeoff mode 
    sleep(20)

    print("in Test LOOP ========================================================================================")
    while True:
        print("In loop")

        print("Just before blocking in test")
        msg = drone_connection.recv_match(type='HEARTBEAT', blocking=True)
        msg_boat = boat_connection.recv_match(type='HEARTBEAT', blocking=True)
        print("after blocking")
        print(msg_boat.base_mode)
        if not boat_connection.motors_armed() & msg_boat.base_mode:
            print("boat not armed, trying to arm") 
            boat_connection.arducopter_arm()
            sleep(1)
            takeoff(boat_connection)
            # boat_connection.mav.command_long_send(boat_connection.target_system, boat_connection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,0,1,4,0,0,0,0,0)# 13 is takeoff mode 
            boatstage = 2

        # if msg_boat.base_mode & boat_connection.motors_armed() and boatstage==1:# msg_boat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        #     print("inside boat is armed")
        #     takeoff(boat_connection)
        #     # sleep(1)
        #     boatstage = 2
        
        if boatstage == 2 and boat_connection.motors_armed(): 


            # Send the MISSION_ITEM_INT message
            sleep(1)
            boat_connection.mav.send(msg)
            sleep(5) 
            boat_connection.mav.send(msg2)
            sleep(10) # give some time for the boat to get into position
            boat_connection.mav.send(msg3)
            drone_connection.mav.command_long_send(drone_connection.target_system, drone_connection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,0,1,10,0,0,0,0,0)
            boatstage = 3 # drone should be on its way to the checkpoint and have a height of -1 when arriving there
            





        # kolla dir(drone_connection) och se om det finns några bra kommandon att använda. 
        
        print("drone connection armed?")
        print(drone_connection.motors_armed())
        if drone_connection.motors_armed() and flightstage == 0: # I think this works
            print("drone is armed")
            # drone_connection.mav.command_long_send(drone_connection.target_system, drone_connection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,        0,1,5,0,0,0,0,0)# 13 is takeoff mode 
            # sleep(2)
            drone_connection.mav.command_long_send(drone_connection.target_system, drone_connection.target_component,mavutil.mavlink.MAV_CMD_DO_SET_SERVO,0,3,1800,0,0,0,0,0)
            print("servos should be set and drone should be armed")
            flightstage = 1 # motrs are armed and flight should be in air
        elif not drone_connection.motors_armed():
            print(drone_connection.motors_armed())
            drone_connection.mav.command_long_send(drone_connection.target_system, drone_connection.target_component,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0)
            print("drone connection armed? in elif")
            print(drone_connection.motors_armed())
            flightstage = 0

        if drone_connection.motors_armed() and flightstage == 1: 
            # Changing mode to auto i should do it so that it only changes flightstage if it has been accepted and confirmed 
            drone_connection.mav.command_long_send(drone_connection.target_system, drone_connection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,0,1,10,0,0,0,0,0)
            # drone_connection.mav.command_long_send(drone_connection.target_system, drone_connection.target_component,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0)



            bmsg1 = drone_connection.mav.mission_item_int_encode(
                drone_connection.target_system,  # Target system ID (0 for broadcast)
                drone_connection.target_component,  # Target component ID (0 for broadcast)
                0,  # Sequence number (1 for the first waypoint)
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # Frame of reference
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  # Command ID for waypoint
                2,  # Current (set to 0 for new waypoints)
                1,  # Autocontinue (set to 0 for new waypoints)
                0,  # Param 1 (not used for waypoints)
                0,  # Param 2 (not used for waypoints)
                0,  # Param 3 (not used for waypoints)
                0,  # Param 4 (not used for waypoints)
                int(latitude * 1e7),  # Param 5 (latitude in degrees * 1e7)
                int(longitude * 1e7),  # Param 6 (longitude in degrees * 1e7)
                int(-1),  # Param 7 (altitude in meters)
                0
            )
            drone_connection.mav.command_long_send(drone_connection.target_system, drone_connection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,0,1,15,0,0,0,0,0)
            drone_connection.mav.send(bmsg1)
            print("message should be sent")
            flightstage = 2
        sleep(2)

    sleep(10)
    drone_process.kill()
    # boat_process.kill()
    # gazebo_process.kill()

