# Autonomous Mobile Robotics Assignment 1
Assignment by Cabe Towers 25708017

## Running this assignment
Before following these instructions please follow instructions to clone and open the workspace in the prerequisite environment setup section

### Install dependencies
This assignment requires some additional python modules to be installed. Run the following commands to navigate to the workspace folder and install dependencies
```
cd /workspaces/ros2-teaching-ws/src/amr_assignment_pkg
pip install -r requirements.txt
```

### Building the project

```
cd /workspaces/ros2-teaching-ws/src
colcon build
source install/setup.bash
```

### Running the assignment
Run the following commands in a new terminal to start the simulation environment and launch the assignment.
#### Launch simulation environment 
```
ros2 launch uol_tidybot tidybot.launch.py world:=level_2_2.world
```
#### Launch rviz (optional)
Once open, please load the config file located in `/workspaces/ros2-teaching-ws/src`
```
rviz2
```
#### Add cubes to the environment
After adding cubes spread them around evenly across the arena
```
ros2 run  uol_tidybot generate_objects --ros-args -p red:=false -p n_objects:=4
ros2 run  uol_tidybot generate_objects --ros-args -p red:=true -p n_objects:=4
```
#### Launch assignment code
```
ros2 launch amr_assignment_pkg assignment_launch.py
```

## Prerequisite environment setup

### Setup your working environment

1. Make sure you have VSCode installed: https://code.visualstudio.com/download
2. Make sure you have the `Docker` and the `Dev Containers` extension in VSCode installed and working: https://code.visualstudio.com/docs/containers/overview and https://code.visualstudio.com/docs/devcontainers/containers
    * ensure docker is working, i.e. try `docker run --rm hello-world` and check it succeeds for your user
3. The docker image used to provide the Development Container is provided by the [L-CAS](https://lcas.lincoln.ac.uk) Container Registry. You must log in to use it. For simple read access, the username and password is public and is username `lcas`, password: `lincoln`. So, to log in do `docker login -u lcas -p lincoln lcas.lincoln.ac.uk` (you should only have to do this once, as the credentials should be cached unless your home directory is wiped).

### Open in VSCode

1. Open repository in VSCode: https://code.visualstudio.com/docs/sourcecontrol/intro-to-git (or any other way you prefer), e.g. click on "Clone Respository" in VSCode:
    ![Alt text](.assets/clone.png)

2. VSCode should prompt you that there is a devcontainer configured and ask if you want to reopen in container. Re-open in the container

### Access the embedded lite Desktop

1. Click on the "Port" in VSCode, find the "novnc" port, right click on it to open the menu, and then choose either "Open in Browser" to open it outside of VSCode or "Preview in Editor" to have it open within VSCode:

   <img width="735" alt="image" src="https://github.com/LCAS/ros2-teaching-ws/assets/1153084/2b0bdfa9-07ea-4238-a0b9-dd2dc8f4c111">

2. (recommended) Set the dekstop scaling by clicking on the settings cog and choose scaling mode "Remote Resizing" if it's not set

   <img width="292" alt="image" src="https://github.com/LCAS/ros2-teaching-ws/assets/1153084/2d9bc88e-7319-4723-968a-0aa08db026ef">

3. click on "Connect" and enter the password `vscode` when prompted:

   <img width="455" alt="image" src="https://github.com/LCAS/ros2-teaching-ws/assets/1153084/ddc224eb-5980-4d9a-994e-b05aa1e9fc1d">


