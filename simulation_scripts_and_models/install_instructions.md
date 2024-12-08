# Uncooperative drone landing simulation setup
This guide is for Windows using Windows Subsystem for Linux (WSL) running Ubuntu. The steps after the WSL installation should be the same for Ubuntu. The links in each section contains install instructions for other operating systems.

## WSL setup 
Setup WSL: https://learn.microsoft.com/en-us/windows/wsl/install
then go to store and install ubuntu or do it via commands. This guide is tested for Ubuntu 22.04.3 LTS from the Microsoft store.
1. Start your ubuntu terminal
2. Follow this guide and enter the commands

## Install git if not installed 
Git install instructions: https://github.com/git-guides/install-git
1. `sudo apt-get update`
2. `sudo apt-get install git-all`
3. `git version`  # Not necessary, this is just to verify that the installation is successful

# Ardupilot 
Here is the ardupilot instructions for different operating systems: https://ardupilot.org/dev/docs/building-the-code.html
This guide will focus on WSL running Ubuntu 22.04.3 LTS. This should also work for ordinary Ubuntu.

## Install Ardupilot
Linux guide: https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux
1. Clone the repo: `git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git`
2. `cd ardupilot`
3. `Tools/environment_install/install-prereqs-ubuntu.sh -y`
4. `. ~/.profile`

## Install gazebo harmonic
General install instructions: https://gazebosim.org/docs/harmonic/install
1. `cd` # to get to home directory
2. `sudo apt-get update`
3. `sudo apt-get install lsb-release wget gnupg`
4. `sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg`
5. `echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null`
6. `sudo apt-get update`
7. `sudo apt-get install gz-harmonic`

## Install Ardupilot gazebo plugin
Following these instructions for gazebo harmonic: https://github.com/ArduPilot/ardupilot_gazebo
1. `sudo apt update`
2. `sudo apt install libgz-sim8-dev rapidjson-dev`
3. `sudo apt install libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl`
4. `echo 'export GZ_VERSION=harmonic' >> ~/.bashrc`
5. `source ~/.bashrc`
6. `git clone https://github.com/ArduPilot/ardupilot_gazebo`
7. `cd ardupilot_gazebo`
8. `mkdir build && cd build` 
9. `cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo`
10. `make -j4`
11. `cd` # to get to home directory 
12. `echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc`
13. `echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc`
14. `source ~/.bashrc`
15. Test that everything works: `gz sim -v4 -r iris_runway.sdf` 

## Cloning uncooperative drone landing repo
1. `git clone https://github.com/SSRS-Innovation/uncooperative-boat-landing.git`
### Moving all the modified files
1. `cd uncooperative-boat-landing/simulation_scripts_and_models`
2. `cp models worlds ~/ardupilot_gazebo/ -r`
4. `cp Tools ~/ardupilot/ -r`

## Install python and necessary dependencies
1. `pip install geopy`
2. `python3 main.py` # This should run the first test scenario.

## In case of Errors
# Waf error 
1. navigate to ardupilot installation folder
2. `./waf clean`
# in case of Protobuf error
pip install protobuf==3.12.4

# How to:s
## How to change camera properties 
This example will be for the model: zephyr_with_ardupilot_and_boundingboxcam
1. Navigate to the folder where the models are it should be ../ardupilot_gazebo/models/zephyr_with_ardupilot_and_boundingboxcam
2. Open the model.sdf file
3. Under the sensor: <sensor name="boundingbox_camera" type="boundingbox_camera"> you can change resolution, FOV, clipping distance and refresh rate of the simulated camera sensor.  

