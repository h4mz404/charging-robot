# ENPM 662 - Introduction to Robot Modeling

## Project-2: Charging Robot

In our project we tried to model a Charging robot. Charging Bot is a unique robotic system that uses advanced robotic manipulation, vision, and trajectory planning techniques to automate the charging process for stationary electric cars (EVs). The technology uses a robot manipulator to dock precisely into an EV's charging port, led by a stationary RGBD (3D) camera that provides real-time perception. This camera helps Charging Bot to accurately determine the position and orientation of the EV's charging port, even in dynamic or congested surroundings. 

## Team Members:

- VENKATA MADHAV TADAVARTHI (121058768) 
- HAMZA SHAH KHAN (119483152) 
- DOMINIC JAMES GAGLIARDI (116010306) 
- LAKSHMI PRAVALLIKA ADIBHATLA (121288165) 

## Installation:

Clone the repository inside your ros2 workspace:

```
mkdir -p ~/ros2_ws/src
cd src
```

```
git clone https://github.com/h4mz404/charging-robot
```

Now come to the workspace location and compile the package:
```
cd ~/ros2_ws
colcon build
```
## Testing:

Source the workspace before running the launch file:
```
source ~/ros2_ws/install/setup.bash
```

You can run the simulation by the following command:
```
ros2 launch charging_robot gazebo.launch.py
```

You can insert the _autocar_ model into the gazebo environment by dragging the model from the _insert_ section.

## Demo Video:

The demo video can be found here: https://drive.google.com/file/d/1gxbCZUCsHsARbwQlj9mxJQYrbfb0MMcd/view?usp=drive_link