This repository contains the Openmanipulator-X and moveit packages.
The robot includes an overhead camera.

Devcontainer instructions:

Install VSCode, devcontainer instruction and Docker.
To use the devcontainer on Windows you have to open a powershell as administator and clone the repo using following command:

    $ git clone -c core.symlinks=true git@github.com:vncprado/OMX-Moveit-devcontainer.git

To launch the tutorial use:

    $ catkin_make
    $ source devel/setup.bash
    $ roslaunch omx_tutorials omx_camera_scene.launch 

After this you should be able to open the browser on localhost:6080 and see the GUI (RViz and Gazebo).  
To run the rest of the tutorial:

    $ rosrun omx_tutorials object_detection.py
    $ rosrun omx_tutorials omx_pick_obj.py


To launch the original system just use:

    $ catkin_make
    $ source devel/setup.bash
    $ roslaunch open_manipulator_controllers joint_trajectory_controller.launch sim:=true 
