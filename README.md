### 本地 1.操作流程
#### 1.1 安装前提
要使用此包，需先安装[Python api](https://github.com/elephantrobotics/pymycobot.git)库。

```bash
pip install pymycobot --user
ros1 noetic
```

#### 1.2安装320m5对应的ros镜像版本

#### 1.3 包的下载与安装

下载包到你的ros工作空间中

```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/jiaweilong66/mycobot_320m5_gazebo.git
$ cd ~/catkin_ws/mycobot_320m5_gazebo
$ git checkout visual
$ cd ..
$ catkin_make
$ source devel/setup.bash
```

#### 1.4 启动步骤
#第一步：
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

#（含视觉抓取）第二步启动程序：
```
source ./devel/setup.bash
rosrun vision_control robot.py
```
