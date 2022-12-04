# IEEE Fall 2022 Autonomous UAV (AUAV) Competition Containers

## Changes made by Enze Jiang
All the new python scripts are in onboard->ws->src->auav_f22->scripts
1. offboard.py is the scripts provided by the original repo, it will publish string info to console. 
2. pub_dronemove.py is the take off scripts which will make the drone in the simulator take off. 
3. sub_rgbd_image.py and sub_sensors.py and test1.py are scripts for getting the drone infomation, such as position, speed, and so on.
4. rover_control.py and rover_planning.py and skidsteer_controller.py are scripts made by Ashvin, they are codes for rover agent. 

### To run the code, simply clone the repo, and use docker compose up same as the original one.
To test drone agent, use ros2 run auav_f22 pub_dronemove.py, it will make the drone take off.

## Submission
To write your solutions, fork this repository.

This repository consists of 4 docker containers and several source directories.
* auav_f22_sim: Simulation with ignition Gazebo Fortress.
* auav_f22_gcs: QGroundControl: Runs ground station.
* auav_f22_px4: PX4 Autopilot: This should match the Firmware on your drone (v1.12.3) do not change.
* auav_f22_onboard: Onboard computer running ROS, add your code here. It should directly deploy to the real drone. Try to keep the image small, it must be < 20G.

The source directories: sim/ws/src, onboard/ws/src are mounted inside the docker images. This allows for
rapid development. We recommend that you do all git version control commands on your host machine.

## Safety Requirements
* Your control code should never arm the vehicle or switch the mode during the competition.
* A manual pilot will set the mode switch and arm your vehicle.
* You must demonstrate kill switch functionality for your vehicle.
You can test out these safety procedures by manually setting the mode and arming in QGroundControl.

## Hardware Requirements
* Ubuntu 22.04 host computer
* NVidia graphics card

This repository uses docker and docker compose:
* https://docs.docker.com/desktop/install/ubuntu/

Docker now natively supports NVidia GPUs. We have tested docker with the following drivers:
```bash
sudo apt install nvidia-driver-515 libnvidia-gl-515-server
```

## Docker Images
This repository consists of 4 docker containers and several source directories.
* auav_f22_sim: Simulation with ignition Gazebo Fortress.
* auav_f22_gcs: QGroundControl: Runs ground station.
* auav_f22_px4: PX4 Autopilot: This should match the Firmware on your drone (v1.12.3) do not change.
* auav_f22_onboard: Onboard computer running ROS, made modifications here.

## Clone git submodules
Before you can build the PX4 Firmware, you will need to clone the submodule. Recursive clone is not
necessary here as the PX4 makefile automatically handles this.
```bash
git submodule update --init
```

## Pull
You do not need to build the images. You can download them from dockerhub with the pull command.
```bash
docker compose pull
```

## Run
To start the simulation, run the following command:
```bash
docker compose up
```

## Build
If you need to rebuild the docker images, you can run:
```bash
docker compose build
```

## Connect to Running Container
If you want to login to a running container you can use. Note that we are starting
all containers using terminator, and you can split the window to get a new console.
```bash
docker compose exec onboard bash
```
