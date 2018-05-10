# curpp
Columbia University Robotics Pick and Place - A wrapper around the Moveit! Pick and Place pipeline

## Setup
Import this folder into the `src` directory of your ros workspace and build your workspace:
```bash
$ catkin build
```

This package requires ipdb, asyncio, and packages that are all included in ros-kinetic-desktop-full

## Usage
```python
import curpp
import rospy
import os
import rospkg

rospy.init_node("curpp_demo")

rospack = rospkg.RosPack()
config = curpp.config.Config(os.path.join(rospack.get_path('curpp'), "configs", "mico_config.yaml"))

skill_manager = curpp.skills.CURPPManager(config)


```