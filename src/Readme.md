# 《多机器人系统与控制》课程实验代码|Experiments of course Multi Robot System and Control

## ROS工作空间创建及编译
```
git clone https://github.com/zixinzhang02/Multi-Robot.git
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


## 打开多机器人Gazebo仿真平台并运行多移动机器人运动控制程序
角度及位置一致性（实验1）
```
roslaunch gazebo_swarm_robot_tb3 gazebo_swarm_robot_5.launch
rosrun gazebo_swarm_robot_tb3 gazebo_swarm_robot_control_angle
rosrun gazebo_swarm_robot_tb3 gazebo_swarm_robot_control_x_position
```
队形控制及变换（实验2）
```
roslaunch gazebo_swarm_robot_tb3 gazebo_swarm_robot_5.launch
rosrun gazebo_swarm_robot_tb3 formation_circle
rosrun gazebo_swarm_robot_tb3 formation_cross
rosrun gazebo_swarm_robot_tb3 formation_triangle
```
队形移动（实验3）
```
roslaunch gazebo_swarm_robot_tb3 gazebo_swarm_robot_6.launch
rosrun gazebo_swarm_robot_tb3 move_formation
rosrun gazebo_swarm_robot_tb3 move_formation_cross
```
队形移动及避障（实验4）
```
roslaunch gazebo_swarm_robot_tb3 gazebo_swarm_robot_7.launch
rosrun gazebo_swarm_robot_tb3 exp4

```
