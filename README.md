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

To replace the original phidgets_drivers sub-package with the modded one
```console
$ cd ~/catkin_ws/src/roboTHIx_X1
$ mv -f phidgets_api ~/catkin_ws/src/phidgets_drivers/
```

# PlayStation 4 Gamepad Assignment
![PlayStation 4 Gamepad](https://game.capcom.com/manual/re3/locale_re3/de/ps4/page/21_3_1.png)
  * **1** - Drive Backwards
  * **2** - Drive Forwards
  * **3** - n.a.
  * **4** - Press to reset the camera to the default orientation
  * **5** - n.a.
  * **6** - n.a.
  * **7** - n.a.
  * **8** - n.a.
  * **9** - n.a.
  * **10** - n.a.
  * **11** - n.a.
  * **12** - n.a.
  * **13** - Move camera (pan & tilt)
  * **SHARE** - n.a.
  * **OPTIONS** - n.a.
  * **Touchpad** - n.a.

# Usage Instructions
```console
$ roslaunch robothix_X1_base robothix_X1_base.launch
```
