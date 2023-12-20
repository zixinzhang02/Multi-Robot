/* 
 * Date: 2021-11-29
 * Description: the basic function
 */

#include <ros/ros.h>


#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// #include <eigen3/Eigen/Core>
// #include <eigen3/Eigen/QR>
// #include <eigen3/Eigen/Geometry>
// #include <eigen3/Eigen/Dense>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Geometry>

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

using std::cout;
using std::endl;

class SwarmRobot{

public:

    // 构造函数，初始化SwarmRobot类
    SwarmRobot(ros::NodeHandle *nh, std::vector<int> swarm_robot_id_);
    
    // 析构函数，释放资源
    ~SwarmRobot();

    // 存储Swarm机器人的ID
    std::vector<int> swarm_robot_id;

    // 机器人数量
    int robot_num;

    // 获取指定机器人的当前位置信息
    bool getRobotPose(int index, std::vector<double> &pose_cur);

    // 获取所有机器人的当前位置信息
    bool getRobotPose(std::vector<std::vector<double> > &swarm_pose_cur);

    // 控制指定机器人的运动，通过线速度和角速度
    bool moveRobot(int index, double v, double w);

    // 控制多个机器人的运动，通过多个机器人的线速度和角速度
    bool moveRobot(std::vector< std::vector<double> > &speed); 

    // 停止指定机器人的运动
    bool stopRobot(int index);

    // 停止所有机器人的运动
    bool stopRobot();

    // 检查速度值是否在指定范围内
    double checkVel(double v, double max_v, double min_v);

    void U2VW(int index, double ux_0, double uy_0, double &v, double &w);

    void moveRobotbyU(int index, double ux_0, double uy_0);

    void moveRobotsbyU(Eigen::VectorXd del_x,  Eigen::VectorXd del_y);

    void Formation(Eigen::VectorXd needed_x, Eigen::VectorXd needed_y, Eigen::MatrixXd lap, double conv_x, double conv_y);

    void MoveFormation(Eigen::MatrixXd Gd, Eigen::MatrixXd lap, double v_x, double v_y);

    void ChangeFormationDirection(double target_direction);

    void ComeDot(int index, double x0, double y0,double &ux,double &uy);

    void GetGxGyGd(Eigen::MatrixXd &Gx, Eigen::MatrixXd &Gy,Eigen::MatrixXd &Gd);

    void GetVelocity(Eigen::MatrixXd Gd0,Eigen::MatrixXd Gd,double *ux,double *uy);

private:

   // 用于监听TF（Transform）变换
   tf::TransformListener tf_listener;

   // ROS发布器，用于发布机器人的速度命令
   ros::Publisher cmd_vel_pub[10];

   // ROS节点句柄
   ros::NodeHandle nh_;

};