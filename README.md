# Setting Up the Clearpath Jackal UGV on ROS 2 Humble
This README provides instructions for making the Clearpath Jackal UGV run on ROS 2 Humble. Currently, the packages for the Jackal do not officially support ROS 2 Humble; therefore, they had to be built from source, and required a lot of debugging.
## Instructions
I recommend starting with a new SSD instead of wiping your old SSD. It is a good idea to have a working system as a backup in case you make mistakes.
#### 1. Installing Ubuntu 22.04 LTS
* Create a bootable USB stick with Ubuntu 22.04 LTS .iso file. Instructions from the Ubuntu website are available [here](https://ubuntu.com/tutorials/create-a-usb-stick-on-ubuntu#1-overview).
* Make sure the Jackal is turned off and insert the USB, mouse, keyboard, and monitor into the USB hub
* Turn on the Jackal. Follow the instructions [here](https://ubuntu.com/tutorials/install-ubuntu-desktop#4-boot-from-usb-flash-drive) to install Ubuntu 22.04 LTS.
* After completing the installation, remove the USB, and the Jackal should turn off automatically. Turn it on again, and you should see the login screen.

#### 2. Setting up the wireless network
I used the NUMSR router in the Northwestern MSR lab. It is already set up, so all you need to do is to connect the Jackal to the NUMSR WiFi network.
#### 3. Installing ROS 2 Humble
* Install ROS 2 Humble on your computer and the Jackal. Instructions can be found [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). 
* When installing ROS 2 packages, I recommend installing `ros-humble-desktop-full`
* Follow the instructions all the way up to "Environment setup"

#### 4. Building packages from source and installing dependencies on the Jackal
All of the packages for the Jackal are not available for ROS 2 Humble, so they need to be built from source.

The packages in the following repositories need to be built from source:
* [jackal](https://github.com/jackal/jackal/tree/foxy-devel)
* [jackal_robot](https://github.com/jackal/jackal_robot/tree/foxy-devel)
* [interactive_marker_twist_server](https://github.com/ros-visualization/interactive_marker_twist_server)
* [micro_ros_setup](https://github.com/micro-ROS/micro_ros_setup)
* [uros/micro-ROS-Agent](https://github.com/micro-ROS/micro-ROS-Agent)
* [uros/micro_ros_msgs](https://github.com/micro-ROS/micro_ros_msgs)
* [wireless](https://github.com/clearpathrobotics/wireless/tree/foxy-devel)

Follow the instructions [here](https://www.clearpathrobotics.com/assets/guides/foxy/jackal/JackalInstallRobotSoftware.html) to update the firmware, build the packages from source, and install dependencies. 

#### Updating the firmware
When updating the firmware, running `sudo apt-get install ros-foxy-jackal-firmware` will not work. Instead, go [here](https://packages.clearpathrobotics.com/stable/ubuntu/pool/main/j/jackal-firmware/) and download `ros-foxy-jackal-firmware_1.0.0-focal_all.deb`. Then run the following in the directory where you have this file:
```
sudo apt install ./ros-foxy-jackal-firmware_1.0.0-focal_all.deb
```
Once you install the firmware, follow the rest of the instructions on updating the firmware.
#### Cloning and building packages
Make sure to clone the micro_ros_setup repositories in the humble branch and everything else in the foxy branch. When sourcing the ROS 2 installation, you should be replacing `foxy` with `humble`. Skip "Installing the Systemd Job."

Please note that the instructions from Clearpath do not tell you to clone the interactive_marker_twist_server repository. To clone it, run:
```
git clone -b foxy-devel https://github.com/ros-visualization/interactive_marker_twist_server.git
```
After cloning the necessary packages, replace `CMakeLists.txt` and `marker_server.cpp` in the interactive_marker_twist_server package, replace `jackal_hardware.cpp` and `jackal_hardware.hpp` in the jackal_robot package, and replace `control.launch.py` in the jackal_control package with the ones in this repository. These files have been modified to work with ROS 2 Humble.

Now, install the following packages using `sudo apt install`:
* ros-humble-velodyne-description
* ros-humble-robot-localization
* ros-humble-imu-filter-madgwick
* ros-humble-twist-mux
* ros-humble-diagnostic-aggregator

#### 5. Building packages from source and installing dependencies on your computer
The packages in the following repositories need to be built from source:
* [jackal_desktop](https://github.com/jackal/jackal_desktop/tree/foxy-devel)
* [jackal_simulator](https://github.com/jackal/jackal_simulator/tree/foxy-devel)

In addition, follow the instructions [here](https://www.clearpathrobotics.com/assets/guides/foxy/jackal/JackalInstallDesktopSoftware.html) to build additional Jackal packages. Make sure to clone the repositories in the foxy-devel branch.
#### 6. Setting up ROS 2 to work between your computer and the Jackal
Add `export ROS_DOMAIN_ID=<YourID>` to the `~/.bashrc` on your computer as well as the Jackal's computer. `<YourID>` can be any number between 0 and 101, inclusive, and the one on your computer should match the one on the Jackal's computer. Make sure your computer is on the NUMSR WiFi network. Once you do all of this, you can easily SSH into the Jackal's computer from your computer by running:
```
ssh -oSendEnv=ROS_DOMAIN_ID jackal@jackal-desktop
```
#### 7. Setting up PS4 controller
The process of pairing the PS4 controller with the Jackal's computer is the following:
* Ensure the PS4 controller is fully charged before starting
* Start the bluetoothctl utility with `sudo bluetoothctl`
* Run `agent on`
* Put the PS4 controller into pairing mode by holding down the "share" and "PS" buttons until the light flashes rapidly. Once the light is flashing rapidly, the following steps must be done before the flashing stops.
* Run `scan on`
* Wait until a device called "wireless controller" appears in the list of detected devices. Then run `scan off`.
* Copy the MAC address of the wireless controller. It should be something like "70:20:84:5E:88:B5".
* Run `trust 70:20:84:5E:88:B5`, but put in the MAC address of your controller
* Run `connect 70:20:84:5E:88:B5`, but put in the MAC address of your controller
* At this point, the LED on the controller should have turned solid blue
* Close bluetoothctl with `exit`

#### 8. Setting up Velodyne VLP-16
In order to use the Velodyne, the Jackal needs to be set up to interface with it over the NUMSR WiFi network. The IP address of the Velodyne is 192.168.1.201.
* Go to Settings -> Network
* Under "Wired," it will tell you which cable is plugged in. Click on the gear icon to the right.
* Go to the IPv4 tab and change the IPv4 Method to "Manual"
* Create and enter an IP address for the ethernet port. It should be something like "192.168.1.XXX".
* Enter the subnet mask, which is 255.255.255.0
* Click "Apply" to save changes
* Toggle off the button next to the cable name and toggle it back on
* Go to http://192.168.1.201. You should be able to see the Velodyne's web interface.

In addition, you need to install the velodyne package. This can be done by running:
```
sudo apt install ros-humble-velodyne
```
## Starting the Jackal
* Turn on the Jackal, enable the motor stop, and press the "M" button
* Pair the PS4 controller with the Jackal's computer via Bluetooth
* SSH into the Jackal from your computer
* Source the workspace
* Run `ros2 launch jackal_robot bringup.launch.py` to drive the Jackal