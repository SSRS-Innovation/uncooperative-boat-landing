
# Starting the gazebo simulation for one drone with camera and CV  
This will run the simulation on the island environment with CV and positioning activated however no boat is present in this simulation.  

1. ### CV_terminal:  
    1. `cd uncooperative-boat-landing/simulation_scripts_and_models/`  
    2. `python sim_positioner.py`  
  
2. ### Gazebo terminal, Open a new terminal and run the commands:  
    1. `cd uncooperative-boat-landing/simulation_scripts_and_models/` # Tror inte denna behövs  
    2. `gz sim zephyr_runway.sdf or gz sim island.sdf`  
A gazebo simulation should startup press the **play button in the bottom left corner** to play the simulation.  

3. ### Ardupilot terminal, Open a new terminal and run the command:  
    1. `sim_vehicle.py -v ArduPlane -f gazebo-zephyr --model JSON --map --console --out 0.0.0.0:14551`  
this will save flight logs in the directory you execute it in.  

Ordinary startup commands:(write in the ardupilot terminal)  
`mode fbwa`  
`arm throttle`  # make sure that this is accepted before setting rc  

`rc 3 1800`  
`circle`  
