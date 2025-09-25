###第一步：
启动仿真环境：
```
source ./devel/setup.bash
roslaunch demo_moveit demo_gazebo.launch 
```

#（无视觉抓取）第二步启动程序：
```
source ./devel/setup.bash
rosrun vision_control demo.py
```

``
###（含视觉抓取）第二步启动程序：
```
source ./devel/setup.bash
rosrun vision_control robot.py
```

