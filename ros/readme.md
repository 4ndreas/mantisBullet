

# ROS and Moveit setup

## Install ROS
  
`lsb_release -a` ubuntu version anzeigen


```bash
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

```bash
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```
```bash
sudo apt update
sudo apt install ros-melodic-desktop-full
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

If you just want to change the environment of your current shell, instead of the above you can type:
```bash
source /opt/ros/melodic/setup.bash
```

```bash
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
```

```bash
sudo apt install python-rosdep
sudo rosdep init
rosdep update
```


## installation moveit

```bash
rosdep update
sudo apt-get update
sudo apt-get dist-upgrade
```

```bash
sudo apt-get install ros-melodic-catkin python-catkin-tools
```

```bash
sudo apt install ros-melodic-moveit
```

create workspace dir
```bash
mkdir -p ~/ws_moveit/src
cd ~/ws_moveit/src
```

#### create a custom package 
+ Erstellt ein eigenes package nötig um z.B. stl files zu finden
```bash
catkin_create_pkg mantisrobot std_msgs rospy roscpp
```

+ Roboter runterladen
```bash
cd ~/ 
git clone https://github.com/4ndreas/mantisBullet
```

`matisBullet/ros/mantis_moveit_config/` nach `~/ws_moveit/src` kopieren


```bash
cd ~/ws_moveit/src
rosdep install -y --from-paths . --ignore-src --rosdistro melodic
```

```bash
cd ~/ws_moveit
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```

```bash
source ~/ws_moveit/devel/setup.bash
```

#### source in bash
sorgt dafür das beim öffen von einem terminal automatisch die sourcen geladen werden

```bash
echo 'source ~/ws_moveit/devel/setup.bash' >> ~/.bashrc
```

kann man mit 
`nano ~/.bashrc`
bearbeiten

#### Test if package is found

`rospack find mantisrobot`

nun sollte eines der erstellen package gefunden werden also der pfad angezeigt werden 
sollte nichts gefunden werden gibt es ein build problem 

#### Failback 
möglicherweise gibt es einen conflict mit dem alten build system dann 


```bash
catkin clean
catkin_make
source ~/ws_moveit/devel/setup.bash
```
und testen ob es geht


#### Testen

```bash
roslaunch mantis_moveit_config demo.launch rviz_tutorial:=true
```
sollte eine rviz mit dem robo öffnen


#### joystick control

```bash
roslaunch mantis_moveit_config demo.launch
```

```bash
roslaunch panda_moveit_config joystick_control.launch
```

`roslaunch panda_moveit_config joystick_control.launch dev:=/dev/input/js1`
+ find device
  
lsusb
lshw    
hwinfo --short

## ROS serial

install 
`sudo apt-get install ros-hydro-rosserial-server`


run as TCP
`roslaunch rosserial_server socket.launch`

http://wiki.ros.org/rosserial_server

list com ports
`dmesg | grep tty`

usb-serial-port
`rosrun rosserial_python serial_node.py /dev/ttyUSB0`

network-tcp 
`rosrun rosserial_python serial_node.py TCP`
 
port used for TCP connection is 11411


# ROS - noetic 
you are f#%§$

http://wiki.ros.org/noetic/Installation/Ubuntu


fix for rosdep?

`sudo apt-get install python3-osrf-pycommon python3-catkin-tools`
`sudo apt install python3-catkin-lint python3-pip`
`pip3 install osrf-pycommon`


https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html#install-ros-and-catkin
