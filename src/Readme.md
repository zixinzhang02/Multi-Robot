# 角度，位置一致性协议

## ROS工作空间创建及编译
新建一个文件夹(如mr_ws)，src放进去，然后在文件夹这个目录下，在终端运行catkin_make进行编译（每次改完代码也记得编译一下）
```shell
catkin_make
```
## 添加环境变量
每次新建一个终端运行ros程序时，需要添加一下环境变量
```
source devel/setup.bash
```
也可以把这句话加进系统的.bashrc文件里，这样每次打开终端都会自动运行这句语句

用vim编辑.bashrc文件
```
vim ~/.bashrc
```
然后把source devel/setup.bash这句话加进去后保存退出


## 打开多机器人Gazebo仿真平台
```
roslaunch gazebo_swarm_robot_tb3 gazebo_swarm_robot_5.launch
roslaunch gazebo_swarm_robot_tb3 gazebo_swarm_robot_6.launch
```

## 运行多移动机器人运动控制程序
角度一致性
```
rosrun gazebo_swarm_robot_tb3 gazebo_swarm_robot_control_angle
```
位置一致性
```
rosrun gazebo_swarm_robot_tb3 gazebo_swarm_robot_control_x_position
```

rosrun gazebo_swarm_robot_tb3 test_2

rosrun gazebo_swarm_robot_tb3 formation_change

rosrun gazebo_swarm_robot_tb3 formation