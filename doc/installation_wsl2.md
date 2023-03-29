# WSL2 Linux Ubuntu Installation

**Note:** usbipd-win project on the Windows host and the USBIP tools on the Linux are required to support the USB devices on WSL2<br>RSUSB_BACKEND is enforced due to the native v4l2 backend does not work in this case. You will still get the frame metadata such as frame timestamp and do not need to run kernel patch scripts
<br><br>

## WSL2 Linux Ubuntu Build Dependencies
The instructions provided in this tutorial was tested based on Windows 11, version 21H2, build 22000.1696 

## Prerequisites
**Install WSL-Ubuntu 22.04**  

  * [Install Linux on Windows with WSL](https://learn.microsoft.com/en-us/windows/wsl/install)
  * Launch Windows PowerShell in administrator mode
  * Install Ubuntu 22.04 Distro
    * `wsl --install -d Ubuntu-22.04`  <br />
  * Restart the machine

**Install USBIP**  

  * Instsall [uspipd-win_*.msi](https://github.com/dorssel/usbipd-win/releases) on the Windows host
  * List all availabled usb devices
    * `usbipd wsl list`
  * attached the realsense device to wsl 
    * `usbipd wsl attach --busid <busid>`
  
  * Instsall USBIP tools on the Linux
    * `sudo apt install linux-tools-generic hwdata`
    * `sudo update-alternatives --install /usr/local/bin/usbip usbip /usr/lib/linux-tools/*-generic/usbip 20`
    * `lsusb | grep 8086`  <br />
    
    ```
    lsusb | grep 8086
    Bus 002 Device 002: ID 8086:0b5c Intel Corp. Intel(R) RealSense(TM) Depth Camera 455
    ```

  
## Install librealsense2 SDK
  * On WSL Ubuntu 22.04, in the PowerShell
    * `sudo apt-get install build-essential cmake git libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev`
    * `git clone https://github.com/IntelRealSense/librealsense.git`
    * `cd librealsense/`
    * `sudo service udev restart`
    * `sudo ./scripts/setupudevrules.sh`
    * `mkdir build && cd build/`
    * `cmake ../ -DBUILDEXAMPLES=true -DCMAKEBUILDTYPE=Release -DFORCERSUSB_BACKEND=true`
    * `make && sudo make install`
     ```
       $rs-enumerate-devices -s 
       Device Name                   Serial Number       Firmware Version
       Intel RealSense D455          035322250187        05.12.13.50*
     ```
  * Done building librealsense 2 SDK. Proceed to the next step for realsense-ros (Optional)


## Install ROS2 on WSL Linux
  * On WSL Ubuntu 22.04, in the PowerShell
    * Install [ROS2 humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
    * `sudo apt install software-properties-common`
    * `sudo add-apt-repository universe`
    * `sudo apt update && sudo apt install curl`
    * `sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg`
    * `echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTUCODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null`
    * `sudo apt update` 
    * `sudo apt upgrade` 
    * `sudo apt install ros-humble-desktop` 
    * `sudo apt install ros-dev-tools`
    * `source /opt/ros/humble/setup.bash`
    

 ## Install realsense-ros from Development Branch
 * On WSL Ubuntu 22.04, in the PowerShel
   * Install [realsense-ros](https://github.com/IntelRealSense/realsense-ros#step-3-install-intel-realsense-ros2-wrapper-from-sources)
   * `mkdir -p ~/ros2_ws/src`
   * `cd ~/ros2_ws/src/ `
   * `git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development `
   * `cd ~/ros2_ws`
   * `sudo apt-get install python3-rosdep -y`
   * `sudo rosdep init`
   * `rosdep update`
   * `rosdep install -i --from-path src --rosdistro $ROSDISTRO --skip-keys=librealsense2 -y`
   * `colcon build`

## Verify Install
* Launch realsense camera
  * `source ~/ros2_ws/install/setup.bash`
  * `ros2 launch realsense2_camera rslaunch.py` 
  * Sample Output
  ```
  [realsense2cameranode-1] [INFO] [camera.camera]: RealSense ROS v4.51.1
  [realsense2cameranode-1] [INFO] [camera.camera]: Built with LibRealSense v2.53.1
  [realsense2cameranode-1] [INFO] [camera.camera]: Running with LibRealSense v2.53.1
  [realsense2cameranode-1] [INFO] [camera.camera]: Device with serial number 035322250187 was found.
  ...
  [realsense2cameranode-1] [INFO] [camera.camera]: Device Name: Intel RealSense D455
  [realsense2cameranode-1] [INFO] [camera.camera]: Device Serial No: 035322250187
  [realsense2cameranode-1] [INFO] [camera.camera]: Device physical port: 2-1-4
  [realsense2cameranode-1] [INFO] [camera.camera]: Device FW version: 05.12.13.50
  [realsense2cameranode-1] [INFO] [camera.camera]: Device Product ID: 0x0B5C
  ...
  [realsense2cameranode-1] [INFO] [camera.camera]: Set ROS param depthmodule.profile to default: 848x480x30 
  [realsense2cameranode-1] [INFO] [camera.camera]: Set ROS param rgbcamera.profile to default: 1280x720x30
  [realsense2cameranode-1] [INFO] [camera.camera]: Set ROS param gyrofps to default: 200 
  [realsense2cameranode-1] [INFO] [camera.camera]: Set ROS param accelfps to default: 63
  ```
  <br />
* Launch another WSL shell
  * `wsl`
* List ROS topics
  * `source /opt/ros/humble/setup.bash`
  * `source ~/ros2_ws/install/setup.bash`
  * `ros2 topics list`
* Sample output of RealSense D455 with default rs_launch.py script
  ```
  /camera/color/camerainfo 
  /camera/color/imageraw
  /camera/color/metadata
  /camera/depth/camerainfo 
  /camera/depth/imagerectraw 
  /camera/depth/metadata
  /camera/extrinsics/depthtocolor 
  /camera/imu 
  /parameterevents
  /rosout
  /tf_static
  ```
  
