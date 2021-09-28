# roboTHIx_X1
These are the packages for the robot platform X1 of the team 'roboTHIx'.
This version supports Noetic. Other versions are not tested so far.

# Preparation
Before you can use the roboTHIx X1 packages, you have to install external depended packages.

  * [`ds4_driver`](https://github.com/naoki-mizuno/ds4_driver) to use a PlayStation 4 gamepad
  * [`phidgets_drivers`](https://github.com/ros-drivers/phidgets_drivers/tree/noetic) to use the Phidget motor drivers (and other sensors in the future). The original phidgets_drivers package has been modded. How you replace the sub-folder **phidgets_drivers** with the **phidgets_drivers** folder from this package will be explaind in the further course of this explanation.
  * [`twist_mux`](https://github.com/ros-teleop/twist_mux) to use and prioritize divers sources of **cmd_vel**

# Installation
To download and install this package, do the following steps
```console
$ cd ~/catkin_ws/src
$ git clone https://github.com/roboTHIx/roboTHIx_X1
```
Finally build your workspace
```console
$ cd ~/catkin_ws
$ catkin_make
```
Instead of `catkin_make` you can also use [`catkin build`](https://catkin-tools.readthedocs.io/en/latest/installing.html) (which in my opinion is a better tool)

# Replace Phidgets Files
To replace the original phidgets_drivers sub-package with the [modded](https://github.com/roboTHIx/thi_modded_phidgets_drivers) one
```console
$ git clone https://github.com/roboTHIx/thi_modded_phidgets_drivers
$ cd thi_modded_phidgets_drivers
$ chmod +x replace_phidgets_files.sh
$ ./replace_phidgets_files.sh
```
After this, you will be asked if you really want to replace the original phidgets_drivers files. If you answer with "Y" or "y", the files in the ~/catkin_ws/src/phidgets_drivers folder will be replaced.
Bild your workspace again
```console
$ cd ~/catkin_ws
$ catkin_make
```

# Usage Instructions
```console
$ roslaunch robothix_x1_base robothix_x1_base.launch
```
