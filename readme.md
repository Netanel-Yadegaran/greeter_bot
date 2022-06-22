# Greeter Bot
Guy Shapira, Netanel Yadegaran

## Step 0 - Follow Previous Project
Follow the guidelines here: https://github.com/arielweizman1/Turtlebot-Autonomous-Navigation-using-Lidar-JetsonNano-and-ROS

You should know how to use the LIDAR, rviz, how to map a room, navigate using a keyboard, and send the robot to a point.

In this project, our robot can: 
* Move autonomously in a predetermined route.
* Recognize people on the way, using lidar-based leg tracking (no camera is used).
* Stop and greet the detected person with an animated face.

## Step 1 - Install OpenCV 3.4
Install OpenCV 3.4 from source.
Commands taken from: https://learnopencv.com/install-opencv3-on-ubuntu/
```
cd ~
git clone https://github.com/opencv/opencv.git
cd opencv
git checkout 3.4
cd ..

git clone https://github.com/opencv/opencv_contrib.git
cd opencv_contrib
git checkout 3.4
cd ..

cd opencv
mkdir build
cd build

sudo apt update
sudo apt install liblapacke-dev checkinstall -y

cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_C_EXAMPLES=ON \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
    -D BUILD_EXAMPLES=ON .. \
    -DENABLE_PRECOMPILED_HEADERS=OFF \
    -DLAPACKE_H_PATH=/usr/include

make -j4
sudo make install
sudo sh -c 'echo "/usr/local/lib" >> /etc/ld.so.conf.d/opencv.conf'
sudo ldconfig

```

## Step 2 - Leg Tracker
Install leg tracker from this repository. This is adapted from https://github.com/angusleigh/leg_tracker.git

Run the minimal, lidar, and leg_tracker in separate shells:
```
roslaunch turtlebot_bringup minimal.launch
roslaunch rplidar_ros view_rplidar.launch
roslaunch leg_tracker joint_leg_tracker.launch
```
Note that the lidar may change its USB number, and therefore not work. Check the number using:
 ```
ll /dev/ | grep lidar
```
Then update `/dev/ttyUSB0` in `rplidar.launch` to the correct number:
```
<param name="serial_port" type="string" value="/dev/ttyUSB0"/>
```
When running the leg tracker, it publishes visualization markers, which can be shown in rviz:
![rviz](/img/rviz_markers.png)

Notice we changed some parameters in the launch file to optimize the detection.

Example of a detection (75 is the id of the person):

![human_detection](/img/human_detection.png)

## Step 3 - Patrol
Install patroller from this repository. This is the main node that will tell the robot where to go, and to greet people.
We used the code from https://github.com/HotBlackRobotics/hotblackrobotics.github.io/blob/master/en/blog/_posts/2018-01-29-seq-goals-py.md

Remember to do `2D Pose Estimate` in rviz, before running the patroller.
## Step 4 - Face
### 4.1 - Install Festival
Clone and make:
```
cd ~
git clone https://github.com/festvox/festival.git
cd festival
make
```
More info in the file `INSTALL`.

### 4.2 - Install EST
```
sudo apt install esound alsa libesd-java audiofile-tools -y
```
add these two lines to `/etc/apt/sources.list`:
```
deb http://ports.ubuntu.com/ubuntu-ports/ xenial universe
deb-src http://ports.ubuntu.com/ubuntu-ports/ xenial universe
```
Then install `libesd0-dev`:
```
sudo apt install libesd0-dev -y
```
Clone the EST repo:
```
cd ~
git clone https://github.com/festvox/speech_tools.git
cd speech_tools
```
Installation instructions are in install.md. What we have done:
```
./configure
```
In the file `config/config` uncomment and set the line:
```
SHARED=1
``` 
Also uncomment:
```
INCLUDE_MODULES += ESD_AUDIO
```
Then compile and test:
```
make
make test
```
### 4.3 - Install Homer Robot Face
Install Homer Robot Face:
```
cd ~/catkin_ws/src
git clone https://github.com/homer-robotics/homer_robot_face.git
```
Before compiling, replace the `h` file with `hpp` in `ImageDisplay.cpp`:
```
// #include <opencv2/core_c.h>
#include <opencv2/core.hpp>
```
Add `${OpenCV_LIBS}` to the `target_link_libraries` commands in `homer_robot_face/CMakeLists.txt`:
```
target_link_libraries(FestivalSynthesizer
	${catkin_LIBRARIES}
	// ... more libraries
	${OpenCV_LIBS} // <--- ADD THIS
	//...
)

target_link_libraries(RobotFace
	${catkin_LIBRARIES}
	// ... more libraries
	${OpenCV_LIBS} // <--- ADD THIS
	//...
)
```
Force the OpenCV version:
```
find_package(OpenCV 3.4 REQUIRED)
```
Then compile:
```
cd ~/catkin_ws
catkin_make
```
If all went according to plan, it should compile successfully.

## Step 5 - Set Patrol Points
Open your map in rviz. Click on the "Publish Point" button at the top. Now you can hover on the map, and see the coordinates in the bottom left.

In the launch file of patroller, set your patrol points as triplets of `x,y,z` (`z` should be 0), and orientation in degrees.

## Step 6 - Run it All!
Create an `.sh` file, and paste this content:
```
gnome-terminal -e "bash -c \"roslaunch turtlebot_bringup minimal.launch\""
gnome-terminal -e "bash -c \"roslaunch rplidar_ros view_rplidar.launch\""
gnome-terminal -e "bash -c \"roslaunch turtlebot_navigation <your_map_file>.launch\""
gnome-terminal -e "bash -c \"roslaunch leg_tracker joint_leg_tracker.launch\""
gnome-terminal -e "bash -c \"roslaunch turtlebot_rviz_launchers view_navigation.launch --screen\""
gnome-terminal -e "bash -c \"roslaunch patroller patroller.launch\""
```
Change `<your_map_file>` to your map launch file.

You can now run this file, and the nodes will launch automatically.

Another option is to launch them one by one in separate shells:
```
roslaunch turtlebot_bringup minimal.launch
roslaunch rplidar_ros view_rplidar.launch
roslaunch turtlebot_navigation <your_map_file>.launch
roslaunch leg_tracker joint_leg_tracker.launch
roslaunch turtlebot_rviz_launchers view_navigation.launch --screen
roslaunch patroller patroller.launch
```
The robot should start walking and greeting people.
