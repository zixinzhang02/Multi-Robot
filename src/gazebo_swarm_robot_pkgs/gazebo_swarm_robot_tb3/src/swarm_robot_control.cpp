/* 
 * Date: 2021-11-29
 * Description: the basic function
 */
/*
这段代码是一个C++程序，它似乎是用于控制多个群体机器人的移动和获取位置信息的类。以下是代码中的主要部分的解释：

#include <swarm_robot_control.h>：这是一个预处理指令，用于包含名为swarm_robot_control.h的头文件，该头文件可能包含了代码中使用的一些库和声明。

SwarmRobot 类的构造函数：

构造函数用于初始化SwarmRobot 类的对象。
ros::NodeHandle *nh 是一个ROS节点句柄指针，它用于与ROS通信。
std::vector<int> swarm_robot_id_ 是一个包含整数的向量，表示每个机器人的唯一标识。
构造函数初始化了一些成员变量，包括robot_num（机器人数量）和一些ROS消息发布器。
SwarmRobot 类的析构函数：这是一个空的析构函数，用于销毁SwarmRobot 类的对象。

getRobotPose 方法：

getRobotPose 方法用于获取指定机器人的位置信息。
它有两个重载版本，一个用于获取单个机器人的位置，另一个用于获取所有机器人的位置。
该方法使用ROS的tf库来查找机器人的位置信息，并将位置信息存储在传入的std::vector<double>中。
moveRobot 方法：

moveRobot 方法用于控制单个机器人的运动，传入的参数包括线速度（v）和角速度（w）。
它使用ROS消息发布器发布机器人的速度控制消息。
stopRobot 方法：

stopRobot 方法用于停止单个机器人的运动，将机器人的线速度和角速度都设置为0。
与moveRobot 方法类似，它也使用ROS消息发布器发布速度控制消息。
checkVel 方法：

checkVel 方法用于检查输入的速度值是否在允许的范围内，并将速度限制在指定的最大和最小值之间。
整个类似乎旨在提供对多个机器人的运动控制和位置信息获取的接口，这对于协调和控制群体机器人非常有用。它还与ROS（机器人操作系统）一起工作，这是一个用于控制机器人的常用框架。
*/

#include <swarm_robot_control.h> // 包含自定义的头文件

// SwarmRobot 类的构造函数
SwarmRobot::SwarmRobot(ros::NodeHandle *nh, std::vector<int> swarm_robot_id_):
    swarm_robot_id(swarm_robot_id_) // 初始化成员变量 swarm_robot_id
{
    this->robot_num = swarm_robot_id.size(); // 计算机器人数量

    std::cout << "robot_num="<< robot_num <<std::endl; // 打印机器人数量

    /* Initialize swarm robot */
    for(int i = 0; i < 10; i++) {
        std::string vel_topic = "/robot_" + std::to_string(i+1) + "/cmd_vel"; // 生成机器人速度控制的 ROS 话题名称
        cmd_vel_pub[i] = nh_.advertise<geometry_msgs::Twist>(vel_topic, 10); // 创建速度控制消息发布器
    }
    // 初始化位置矩阵
    this->distance_matrix.resize(this->robot_num, this->robot_num);
    this->distance_matrix.setZero();
    this->x_distance_matrix.resize(this->robot_num, this->robot_num);
    this->x_distance_matrix.setZero();
    this->y_distance_matrix.resize(this->robot_num, this->robot_num);
    this->y_distance_matrix.setZero();

    this->ux.resize(this->robot_num);
    this->ux.setZero();
    this->uy.resize(this->robot_num);
    this->uy.setZero();

}

SwarmRobot::~SwarmRobot()
{
    // 析构函数为空
}

/* 获取单个机器人的位置信息 */
bool SwarmRobot::getRobotPose(int index, std::vector<double> &pose_cur) {
    pose_cur.resize(3,0.0); // 初始化位置信息向量

    tf::StampedTransform transform;
    std::string robot_frame = "robot_" + std::to_string(this->swarm_robot_id[index]) + "/base_footprint";
    std::string base_marker = "robot_" + std::to_string(this->swarm_robot_id[index]) + "/odom";

    // 尝试获取机器人的位置信息
    try{
        this->tf_listener.waitForTransform(base_marker, robot_frame, ros::Time(0), ros::Duration(0.5));
        this->tf_listener.lookupTransform(base_marker, robot_frame, ros::Time(0), transform);
    }
    catch(tf::TransformException ex) {
        ROS_ERROR("%s",ex.what()); // 输出错误消息
        return false; // 返回失败
    }

    tf::Quaternion q = transform.getRotation();
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // 获取位置信息并存储在 pose_cur 中
    pose_cur.resize(3);
    pose_cur[0] = transform.getOrigin().x();
    pose_cur[1] = transform.getOrigin().y();
    pose_cur[2] = yaw;

    ROS_INFO_STREAM("Get pose of robot_" << swarm_robot_id[index] << " is: x=" << pose_cur[0] << " y=" << pose_cur[1] << " theta=" << pose_cur[2]);
    return true; // 返回成功
}

/* 获取所有机器人的位置信息 */
bool SwarmRobot::getRobotPose(std::vector< std::vector<double> > &current_robot_pose) {
    current_robot_pose.resize(this->robot_num); // 初始化位置信息向量的大小
    std::vector<bool> flag_pose(this->robot_num, false); // 初始化位置获取状态标志

    bool flag = false;

    while(! flag) {
        flag = true;
        for(int i = 0; i < this->robot_num; i++) {
            flag = flag && flag_pose[i]; // 检查是否已获取所有机器人的位置信息
        }
        for(int i = 0; i < this->robot_num; i++) {
            std::vector<double> pose_robot(3);
            if(getRobotPose(i, pose_robot)) { // 获取机器人的位置信息
                current_robot_pose[i] = pose_robot;
                flag_pose[i] = true; // 标记位置信息已获取
            }
        }
    }
    ROS_INFO_STREAM("Succeed getting pose!"); // 输出成功消息
    return true;   
}

/* 控制单个机器人的移动 */
bool SwarmRobot::moveRobot(int index, double v, double w) {
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = v;
    vel_msg.angular.z = w;
    cmd_vel_pub[swarm_robot_id[index]-1].publish(vel_msg); // 发布速度控制消息
    ROS_INFO_STREAM("Move robot_" << swarm_robot_id[index] << " with v=" << v << " w=" << w);
    return true; // 返回成功
}

/* 控制所有机器人的移动 */
bool SwarmRobot::moveRobot(std::vector< std::vector<double> > &speed) {
  if(this->robot_num != speed.size())
  {
    ROS_INFO_STREAM("The robot number does not equal the speed number!"); // 输出错误消息
    return false; // 返回失败
  }

  for(int i=0; i<this->robot_num; i++)
  {
    if(!this->moveRobot(this->swarm_robot_id[i],speed[i][0],speed[i][1])) { // 控制单个机器人的移动
      return false; // 返回失败
    }
  }
  return true; // 返回成功
}

/* 停止单个机器人的运动 */
bool SwarmRobot::stopRobot(int index) {
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0.0;
    vel_msg.angular.z = 0.0;
    cmd_vel_pub[swarm_robot_id[index]-1].publish(vel_msg); // 停止单个机器人的运动
    ROS_INFO_STREAM("Stop robot_" << swarm_robot_id[swarm_robot_id[index]]);
    return true; // 返回成功
}

/* 停止所有机器人的运动 */
bool SwarmRobot::stopRobot() {
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0.0;
    vel_msg.angular.z = 0.0;
    for(int i = 0; i < this->robot_num; i++) {
        cmd_vel_pub[this->swarm_robot_id[i]-1].publish(vel_msg); // 停止所有机器人的运动
    }
    ROS_INFO_STREAM("Stop all robots."); // 输出消息
    return true; // 返回成功
}

/* 检查速度值是否在允许的范围内 */
double SwarmRobot::checkVel(double v, double max_v, double min_v) {
    if(max_v <= 0 || min_v <= 0) {
        std::cout << "Error input of checkW()" << std::endl; // 输出错误消息
        return v;
    }
    
    if(v > 0) {
        v = std::max(v, min_v); // 限制速度的最小值
        v = std::min(v, max_v); // 限制速度的最大值
    } else {
        v = std::min(v, -min_v); // 限制反向速度的最小值
        v = std::max(v, -max_v); // 限制反向速度的最大值
    } 
    return v;
}

/* 随机初始化（单个）机器人的角度 */
bool SwarmRobot::RandomInitialize(int index) {
    geometry_msgs::Twist vel_msg;
    double w_random = rand() % 10;
    w_random /= 10;
    double v = 0;
    vel_msg.angular.z = w_random;
    cmd_vel_pub[swarm_robot_id[index]-1].publish(vel_msg); // 发布速度控制消息
    ROS_INFO_STREAM("Randomly Initializing Robot_" << swarm_robot_id[index] << " with v=" << v << " w=" << w_random);
    return true; // 返回成功
}

void SwarmRobot::U2VW(int index, double ux_0,double uy_0, double &v, double &w){
    std::vector<double> pose_cur;
    getRobotPose(index, pose_cur);
    double theta_robot = pose_cur[2];
    double ux = ux_0 * std::cos(theta_robot) - uy_0 * std::sin(theta_robot);
    double uy = ux_0 * std::sin(theta_robot) + uy_0 * std::cos(theta_robot);

    if (ux == 0) {
        ux = 0.001;
    }
    if (uy == 0) {
        uy = 0.001;
    }
    
    double theta2 = ux/uy;
    double T = 0.05;
    v = (ux*ux + uy*uy)/ux * std::atan(theta2);
    w = 0.2*((2/T)*std::atan(theta2));
    // while (w > 2*3.1416)
    // {
    //     w = w - 2*3.1416;
    // }
    // if (w > 3.1416) {
    //     w = -3.1416 * 2 + w;
    // }
    // w *= 0.3;
    // 限制w的范围在[-1,1]
    if (w > 0.3) {
        w = 0.3;
    }
    if (w < -0.3) {
        w = -0.3;
    }
    // w *= 0.2;

    
}

void SwarmRobot::moveRobotbyU(int index, double ux_0, double uy_0){
    double v,w;
    U2VW(index, ux_0, uy_0, v, w);
    moveRobot(index, v, w);
}

void SwarmRobot::calculate_all_Distance(){
    // update all the distance between robots
    // store it into the distance matrix
    for (int i = 0; i < this->robot_num; i++) {
        for (int j = 0; j < this->robot_num; j++) {
            if (i == j) {
                this->distance_matrix(i, j) = 0;
            } else {
                std::vector<double> pose_i;
                std::vector<double> pose_j;
                getRobotPose(i, pose_i);
                getRobotPose(j, pose_j);
                double distance_matrix_ij = sqrt(pow(pose_i[0] - pose_j[0], 2) + pow(pose_i[1] - pose_j[1], 2));
                this->distance_matrix(i, j) = distance_matrix_ij;
            }
        }
    }
}

void SwarmRobot::calculate_all_x_distance(){
    // update all the distance between robots in x axis
    // store it into the y_distance_matrix
    for (int i = 0; i < this->robot_num; i++) {
        for (int j = 0; j < this->robot_num; j++) {
            if (i == j) {
                this->x_distance_matrix(i, j) = 0;
            } else {
                std::vector<double> pose_i;
                std::vector<double> pose_j;
                getRobotPose(i, pose_i);
                getRobotPose(j, pose_j);
                double distance_matrix_ij = pose_i[0] - pose_j[0];
                this->x_distance_matrix(i, j) = distance_matrix_ij;
            }
        }
    }
}

void SwarmRobot::calculate_all_y_distance(){
    // update all the distance between robots in y axis
    // store it into the y_distance_matrix
    for (int i = 0; i < this->robot_num; i++) {
        for (int j = 0; j < this->robot_num; j++) {
            if (i == j) {
                this->y_distance_matrix(i, j) = 0;
            } else {
                std::vector<double> pose_i;
                std::vector<double> pose_j;
                getRobotPose(i, pose_i);
                getRobotPose(j, pose_j);
                double distance_matrix_ij = pose_i[1] - pose_j[1];
                this->y_distance_matrix(i, j) = distance_matrix_ij;
            }
        }
    }
}

void SwarmRobot::HardGraph2Speed(Eigen::MatrixXd Gx0, Eigen::MatrixXd Gy0)
{
    // this->x_distance_matrix
    // this->y_distance_matrix
    // this->distance_matrix
    
    Eigen::MatrixXd gx = Gx0 - this->x_distance_matrix;
    Eigen::MatrixXd gy = Gy0 - this->y_distance_matrix;

    for (int i = 0; i < swarm_robot_id.size(); i++)
    {
        for (int j = 0; j < swarm_robot_id.size(); j++)
        {
            double T = 5000;
            this->ux(i) += gx(i, j) / T;
            this->uy(i) += gy(i, j) / T;
        }
    }
}
