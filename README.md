# Senseglove ros2_workspace
A workspace for the integration of the SenseGlove into ROS2 galactic.
This workspace makes use of ros_control for automatically initiating publisher and subscriber nodes for the state of the senseglove.

## 1. For ROS beginners: ##
If you are totally unfamiliar with ROS we advise you to take a look at the ROS-wiki for a quick startup guide.
We especially recommend the following tutorials:
* http://wiki.ros.org/galactic/Installation/Ubuntu

A large part of understanding/working with ROS is the ability to search for the issues you are dealing with.
In our experience, simply googling for an error or issue you have can do the trick. Do keep in mind that there exists a
vast amount of documentation, though it can be tedious to sort through its information.

### setting up the workspace for the senseglove: ###
1. Make sure you are on a Ubuntu 20 system.
2. Download ros-galactic as described in the ros-wiki.
3. clone our workspace by either using `git clone https://github.com/Adjuvo/senseglove_ros_ws.git` or through the webbrowser here on the Github website.
4. Checkout the ROS2 branch
5. Install the following dependencies:
    1. `sudo apt-get install ros-galactic-ros2-control`
    2. `sudo apt-get install ros-galactic-ros2-controller`
6. Run: `rosdep update`
7. navigate, in the terminal, to the workspace folder
    1.  source ros2 workspace: `source /opt/ros/galactic/setup.bash`
    2.  Run: `rosdep install --from-paths src --ignore-src -r -y`
    3.  Build your workspace: `colcon build`
    4.  after building, you can source the workspace itself by using `source install/local_setup.sh`


## 2. General Usage: ##
This repository is meant to present a solid foundation for using the senseglove in ROS galactic. As such it provides no
concrete example projects. It does however provide the user with a few launch files, hardware.launch and senseglove_demo.launch
which initiate the sensegloves.
The senseglove_hardware_interface nodes which are called by these launch files do nothing more than using the senseglove API
in a ROS /ros_control "sanctioned" manner.

__Important:__ Run the sensecom application, present in the SenseGlove_API folder. Otherwise none of the launch files will work !

Users are advised to develop their own applications outside this package and make use of the provided topics. If users do find the need to 
write additions to this package, beware that this repository is still subject to changes and pulling this repo again might override your own code.

**If you, as a user, find a bug or have an issue with getting the workspace up and running, we suggest you leave this as an issue on this repository.**
This practice will allow others to troubleshoot their own problems quicker.

### Example; using two sensegloves in ROS: ###
1. source your workspace
2. make sure your sensegloves are connected through usb or bluetooth
    1. if you checked your connection with sensecom, be sure to exit the application before proceeding
3. run: `ros launch senseglove_launch senseglove_hardware_demo.launch.py`

If all is well, your invocation of the ros launch command should have started all necessary nodes providing intefaces to the senseglove.
In a second (properly sourced) terminal you can verify that these nodes are publishing by invoking: ros topic list
you can further test the application by checking that these topics get published by invoking: ros topic echo /topic_name
Each gloves run in their own namespace with a common static publisher to world for visualizing preference. Thoses static tf publisher can be removed from `hardware.launch.py` when using with other robot.
