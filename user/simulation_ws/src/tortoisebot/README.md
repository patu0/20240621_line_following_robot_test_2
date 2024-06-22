# Tortoisebot ROS2 Humble Release

# ![TortoiseBot Banner](https://github.com/rigbetellabs/tortoisebot_docs/raw/master/imgs/packaging/pack_front.png)

![stars](https://img.shields.io/github/stars/rigbetellabs/tortoisebot?style=for-the-badge)
![forks](https://img.shields.io/github/forks/rigbetellabs/tortoisebot?style=for-the-badge)
![watchers](https://img.shields.io/github/watchers/rigbetellabs/tortoisebot?style=for-the-badge)
![repo-size](https://img.shields.io/github/repo-size/rigbetellabs/tortoisebot?style=for-the-badge)
![contributors](https://img.shields.io/github/contributors/rigbetellabs/tortoisebot?style=for-the-badge)

---

# 1. Installation

Use this branch to launch in robot, prepared for humble ROS 2 distribution.

## 1.1 Required Dependences:

```
sudo apt install ros-humble-joint-state-publisher ros-humble-robot-state-publisher ros-humble-cartographer ros-humble-cartographer-ros ros-humble-map ros-humble-gazebo-plugins ros-humble-teleop-twist-keyboard  ros-humble-teleop-twist-joy ros-humble-xacro ros-humble-nav2* ros-humble-urdf ros-humble-rviz

```

```
cd ~/your workspace
colcon build
```

## 1.2 Clone this repo

Make sure you clone the repo in your robot and your remote PC

```
git clone -b ros2-humble --recursive https://bitbucket.org/theconstructcore/tortoisebot.git
```

```
cd ~/your workscpace
colcon build
```

# 2. Setup

- Run `bringup.launch.py` to only spawn the robot
- Run `autobringup.launch.py` to spawn the robot with navigation and slam/localization
- Launch the files with `use_sim_time:=False` when working on real robot

### 2.1 Launching the real robot

- **Simple bringup:**

```
ros2 launch tortoisebot_bringup bringup.launch.py
```

- Launch **robot with SLAM + Nav2:**

```
ros2 launch tortoisebot_bringup autobringup.launch.py use_sim_time:=False exploration:=True
```

- Visualize map creation in `rviz2`:

```
ros2 launch tortoisebot_description rviz.launch.py
```

- Once launched, try sending a Nav2 Goal through `rviz2` in order to create the map.
- Teleop can be used instead as well: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
- Save map (in an appropriate path) with:

```
ros2 run nav2_map_server map_saver_cli -f [map_name]
```

- Launch **robot with Map Server + Nav2** once map is created and saved:

```
ros2 launch tortoisebot_bringup autobringup.launch.py use_sim_time:=False exploration:=False map:=/home/tortoisebot/[path_to_map_file]/[map_name].yaml
```

Note that the above does not launch any type of localization, so **make sure the robot is approximately at the same spot where the robot was when mapping started.**

### 2.2 Launching robot simulation

- **Simple bringup:**

```
ros2 launch tortoisebot_bringup simulation.launch.py
```

- Launch robot simulation with SLAM + Nav2:

```
ros2 launch tortoisebot_bringup autobringup.launch.py use_sim_time:=True
```

- `slam:=False` for map-based navigation.
- The same actions as 2.1 can be taken now to save and use map.

### 2.3 Launch files for reference

#### SLAM

- cartographer.launch.py

#### Navigation

- navigation.launch.py

#### Rviz

- rviz.launch.py

#### Gazebo

- gazebo.launch.py

# The TortoiseBot 🐢🤖

The ReadMe is divided into several sections as per different topics and is constantly been updated and maintained with new updates by our talented and dedicated 👥 Team at RigBetel Labs LLP. So don't forget to often come here and check on it for the latest and greatest software updates, projects & skills for your TortoiseBot. Also don't forget to 🌟 Star this repository on top-right corner of the screen to show your 💖 Love and Support 🤗 for our Team. 🤩 It will make us happy and encourage us to make and bring more such projects for you. 😍 Click [here](https://github.com/rigbetellabs/tortoisebot/wiki/1.-Getting-Started) to get started.

1. [Getting Started](https://github.com/rigbetellabs/tortoisebot/wiki/1.-Getting-Started)
2. [Hardware Assembly](https://github.com/rigbetellabs/tortoisebot/wiki/2.-Hardware-Assembly)
3. [TortoiseBot Setup](https://github.com/rigbetellabs/tortoisebot/wiki/3.-TortoiseBot-Setup)
4. [Server PC Setup](https://github.com/rigbetellabs/tortoisebot/wiki/4.-Server-PC-Setup)
5. [Running Demos](https://github.com/rigbetellabs/tortoisebot/wiki/5.-Running-Demos)

[Join](https://discord.gg/qDuCSMTjvN) our community for Free. Post your projects or ask questions if you need any help.

## TortosieBot is sourced, assembled, made & maintained by our team 🧑🏻‍🤝‍🧑🏻 at<br>

RigBetel Labs LLP®, Charholi Bk., via. Loheagaon, Pune - 412105, MH, India 🇮🇳<br>
🌐 [RigBetelLabs.com](https://rigbetellabs.com) 📞 [+91-8432152998](https://wa.me/918432152998) 📨 getintouch.rbl@gmail.com , info@rigbetellabs.com <br>
[LinkedIn](http://linkedin.com/company/rigbetellabs/) | [Instagram](http://instagram.com/rigbetellabs/) | [Facebook](http://facebook.com/rigbetellabs) | [Twitter](http://twitter.com/rigbetellabs) | [YouTube](https://www.youtube.com/channel/UCfIX89y8OvDIbEFZAAciHEA) | [Discord Community](https://discord.gg/qDuCSMTjvN)
