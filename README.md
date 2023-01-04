# Multiple Proactive Behavior Learning (MPL)

![control](picture/control.png)

Project page: https://ens-lab.sdu.dk/loft-project/

# Simulation

static environment            |  dynamic environment
:-------------------------:|:-------------------------:
![static_simulation](picture/static_simulation.gif) |   ![dynamic_simulation](picture/dynamic_simulation.gif) 



# Setup

### Setup: Planning and Neural Control
- build the projects and add the following command to ``` ~/.bashrc ```

add ros path
```
source /opt/ros/noetic/setup.bash
```

add project directory (i.e., the directory where LOFT folder locates). For example, ``` /home/zubuntu/Projects ```
```
export PROJECT_PATH = /home/zubuntu/Projects
```

add project directory
```
source $PROJECT_PATH/LOFT/software/projects/loft_alpha/dwa_planner/catkin_ws/devel/setup.bash --extend
source $PROJECT_PATH/LOFT/software/projects/loft_alpha/robot_control/catkin_ws/devel/setup.bash --extend
source $PROJECT_PATH/LOFT/software/projects/loft_alpha/neural_control/catkin_ws/devel/setup.bash --extend
source $PROJECT_PATH/LOFT/software/projects/loft_alpha/joystick/catkin_ws/devel/setup.bash --extend
source $PROJECT_PATH/LOFT/software/projects/loft_alpha/socket/catkin_ws/devel/setup.bash --extend
```

### Setup: Human Detection

### Setup: Robot Interface

# Running

### Running: ROS setup
- Initialize ros systems (one on the Jetson and another on the PC). Note that two ros systems communicate through pysocket.

### Running: Human Detection
- Start the human detection programs (DTU) and the publisher socket (SDU).

### Running: Planning and Neural Control (Multiple Proactive Behavior Learning; MPL)

- Start the coppeliasim scene
- Start the planning and neural control (SDU) by open the rqt gui using the following command.
```
cd perspect
rqt --perspective-file "loft.perspective"
```
- The following window will pop up.
![rqt_ui](picture/rqt_ui.png)
- Modify the launch file located in ```software/projects/loft_alpha/robot_control/catkin_ws/src/robot_control/launch/ ```.
- On the "ROS Launch GUI" tab, select "robot_control" package and "loft.launch" file. Click "START ALL" to run the launch file.


### Running: Robot Interface
- Start the robot interface.


# Results

### Corridor Simulation

![smoothness_result_corridor_environment_5hz](picture/smoothness_result_corridor_environment_5hz.png)


### Dynamic Simulated Environment

![collision_result_dynamic_environment_2hz](picture/collision_result_dynamic_environment_2hz.png)
