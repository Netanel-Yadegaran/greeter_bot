# Greeter Bot
Guy Shapira, Netanel Yadegaran

## Step 0 - Follow Previous Project
This project is a continuation of a previous project by Ariel Weizman and Shir Erdreich. Follow their guidelines here: https://github.com/arielweizman1/Turtlebot-Autonomous-Navigation-using-Lidar-JetsonNano-and-ROS

You should know how to use the LIDAR, rviz, how to map a room, navigate using a keyboard, and send the robot to a point.

In this project, our robot can: 
* Move autonomously in a predetermined route.
* Recognize people on the way, using lidar-based leg tracking (no camera is used).
* Stop and greet the detected person with an animated face.

## Step 1 - Install OpenCV 3.4
OpenCV is a computer vision and image proccessing library. It is used in a lot of packages, specifically in the leg_tracker and homer_face packages.
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
Festival is a speech proccessing and synthesis library. It is used in homer_face package. 
Clone and make:
```
cd ~
git clone https://github.com/festvox/festival.git
cd festival
make
```
More info in the file `INSTALL`.

### 4.2 - Install EST
EST is short for Edinburgh Speech Tools. It is also used in homer_face package.
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

### Step 4.4 - Disable Screen Blanking and Sidebar
For the best experience, disable the screen from going blank after a minute, so you can see the face. Go to `System Settings > Brightness and Lock` and set the setting to `Never`.  

Additionally, disable the sidebar on the left by going to `System Settings > Appearance > Behavior` and enable `Auto-hide the Launcher`.

### Step 4.5 - Enlarge Face
We maximized the face to better hide the desktop in the background.

In the file: `main.cpp` (in Homer Robot Face source), add this line:
```
window.setWindowState(Qt::WindowMaximized);
```
Before this line:
```
window.show();
```
## Step 5 - Set Patrol Points
Open your map in rviz. Click on the "Publish Point" button at the top. Now you can hover on the map, and see the coordinates in the bottom left. You may also click on the map, and the point will be published to the `/clicked_point`. Subscribe to this topic first to capture the point:
```
rostopic echo /clicked_point
```

In the launch file of patroller, set your patrol points as triplets of `x,y,z` (`z` should be 0), and orientation in degrees (one angle for each triplet `x,y,z`).

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

## Step 7 - Configurations
Many settings for the navigation are located in `~/catkin_ws/src/turtlebot_apps/turtlebot_navigation/param/`.

It is possible to control the robot's speed, both linear and angular, in `dwa_local_planner_params.yaml`.

You can also control costmap settings in these files:
* `costmap_common_params.yaml` - For parameters common to both local and global costmaps.
* `global_costmap_params.yaml` - For the global costmap.
* `local_costmap_params.yaml` - For the local costmap.

Selection of global and local planners is done in `move_base_params.yaml`.

Visualization of the costmaps in rviz:
![rviz costmaps](/img/rviz_costmaps_annotated.png)

## Video Demo
Video showing a detection of a person in rviz, and another showing the robot's face when detecting, can be found in the `vid` folder.

## Future Work
### Leg Tracking
There are some false positives when detecting legs. Possible fixes could be:
* Playing around with the leg tracker parameters.
* Using a camera for facial recognition, in addition to the leg detection.
* Using a better LIDAR, with a higher frequency and resolution, and less noise.

Having said that, it tracks real people well. We believe better results can be achieved by using a camera for the initial recognition, and use the leg tracker for tracking.

### Body Movement
The robot has several motors at the base of the screen. It's possible to rotate the screen around the z axis and to tilt it. Using these, it's possible to rotate the face of the robot towards the detected person, to give it a more welcoming feeling. It may also help to point the camera towards people faces in order to detect and/or recognize them.

We tried to rotate the whole robot towards the detected person, however it didn't work for us. You may look at the `rotate_to_person` in `patroller.py`.

### People Recognition
In this project, we only detected people. It is possible to use a camera to then recognize the person, and greet them more personally.

### Add Speech
The Homer Robot Face package has a sister package called Homer TTS. It can be set up to synthesize the speech from the text shown on screen. If you go this route, note our patroller package publishes the `talking_finished` topic to signal the robot face it has finished talking. You must disable this, and let TTS do it instead. See the `talking_finished` publisher in `patroller.py`.

## References
* Leg Tracker: A. Leigh, J. Pineau, N. Olmedo and H. Zhang, Person Tracking and Following with 2D Laser Scanners, International Conference on Robotics and Automation (ICRA), Seattle, Washington, USA, 2015. [Link to PDF](https://www.cs.mcgill.ca/~aleigh1/ICRA_2015.pdf).

* Homer Face: Seib, Victor and Giesen, Julian and Grüntjens, Dominik and Paulus, Dietrich, Enhancing human-robot interaction by a robot face with facial expressions and synchronized lip movements, 2013, Václav Skala-UNION Agency. [Link to PDF](https://userpages.uni-koblenz.de/~agas/Documents/Seib2013EHI.pdf)