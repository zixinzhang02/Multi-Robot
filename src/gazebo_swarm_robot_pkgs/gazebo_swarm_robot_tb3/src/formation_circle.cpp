/*initialize the robot speed*/
/* 
 * Date: 2021-11-29
 * Description: to the same angle
*/

/*
这段代码的主要功能是实现一种群体机器人的集体控制，使其达到一致的角度，同时限制了机器人的角速度和线速度，以及角度的变化阈值。程序在一个循环中不断检查角度是否已经收敛，如果未收敛则调整机器人的角速度，直到达到一致。最后，程序停止所有机器人的运动并输出成功消息。
*/
#include <swarm_robot_control.h> // 包含自定义的头文件
#include <time.h>
#include <math.h>
#include <Eigen/Dense> // 包含Eigen库
#define pi acos(-1)

/* 主函数 */
int main(int argc, char** argv) {
    clock_t start, end;
    start = clock();
    ros::init(argc, argv, "swarm_robot_control_formation"); // 初始化ROS节点，节点名称为 "swarm_robot_control_formation"
    ros::NodeHandle nh; // 创建ROS节点句柄

    /* 第一步：基于Aruco标记设置群体机器人的ID */
    std::vector<int> swarm_robot_id{1, 2, 3, 4, 5}; // 创建包含群体机器人ID的向量

    /* 初始化群体机器人对象 */
    SwarmRobot swarm_robot(&nh, swarm_robot_id); // 创建 SwarmRobot 类的对象，用于控制一组机器人


    /* 设置拉普拉斯矩阵 */
    Eigen::MatrixXd lap(swarm_robot_id.size(), swarm_robot_id.size()); // 创建拉普拉斯矩阵对象
    lap <<  4, -1, -1, -1, -1,
            -1, 4, -1, -1, -1,
            -1, -1, 4, -1, -1,
            -1, -1, -1, 4, -1,
            -1, -1, -1, -1, 4;
    // lap <<  1, 1,  0,  0,  0,
    //         1, 3, 1,  0, 1,
    //         0, 1,  3, 1, 1,
    //         0,  0, 1,  2, 1,
    //         0, 1, 1, 1,  3; 

    // Eigen::MatrixXd Gx0(swarm_robot_id.size(), swarm_robot_id.size()); // 创建Gx0矩阵对象
    // Eigen::MatrixXd Gy0(swarm_robot_id.size(), swarm_robot_id.size()); // 创建Gy0矩阵对象
    // Gx0 <<  0, -1, -1, -1, -2,
    //         1, 0, 0, 0, -1,
    //         1, 0, 0, 0, -1,
    //         1, 0, 0, 0, -1,
    //         2, 1, 1, 1, 0;
    // Gy0 <<  0, -1, 0, 1, 0,
    //         1, 0, 1, 2, 1,
    //         0, -1, 0, 1, 0,
    //         -1, -2, -1, 0, -1,
    //         0, -1, 0, 1, 0;
    /* 收敛阈值 */
    double conv_th = 0.05;  // 角度的阈值，单位弧度
    double conv_x = 0.05;  // x的阈值，单位m
    double conv_y = 0.05;  // y的阈值，单位m

    /* 速度比例和阈值 */
    double MAX_W = 1;       // 最大角速度（弧度/秒）
    double MIN_W = 0.05;    // 最小角速度（弧度/秒）
    double MAX_V = 0.2;     // 最大线性速度（米/秒）
    double MIN_V = 0.01;    // 最小线性速度（米/秒）
    double k_w = 0.1;       // 角速度的缩放比例
    double k_v = 0.1;       // 线性速度的缩放比例

    double v,w; // 机器人的线速度和角速度

    /* 移动机器人的姿态和下一个姿态 */
    Eigen::VectorXd cur_x(swarm_robot_id.size()); // 当前x坐标
    Eigen::VectorXd cur_y(swarm_robot_id.size()); // 当前y坐标
    Eigen::VectorXd cur_theta(swarm_robot_id.size()); // 当前角度
    Eigen::VectorXd del_x(swarm_robot_id.size()); // x坐标的变化
    Eigen::VectorXd del_y(swarm_robot_id.size()); // y坐标的变化
    Eigen::VectorXd del_theta(swarm_robot_id.size()); // 角度的变化
    Eigen::VectorXd v_theta(swarm_robot_id.size()); // 速度的角度

    /*编队*/
    Eigen::VectorXd needed_x_cross(swarm_robot_id.size());
    Eigen::VectorXd needed_y_cross(swarm_robot_id.size());

    Eigen::VectorXd needed_x_circle(swarm_robot_id.size());
    Eigen::VectorXd needed_y_circle(swarm_robot_id.size());

    Eigen::VectorXd needed_x_triangle(swarm_robot_id.size());
    Eigen::VectorXd needed_y_triangle(swarm_robot_id.size());

    needed_x_cross << 1, 0, 0, 0, -1;
    needed_y_cross << 0, -1, 0, 1, 0;

    needed_x_circle << 2, 0, -2, -1, 1;
    needed_y_circle << -1, -2.5, -1, 1, 1;

    // 计算needed_x_circle[0] 和 needed_y_circle[0]的距离
    double original_radius = std::sqrt(std::pow(needed_x_circle[0], 2) + std::pow(needed_y_circle[0], 2));
    double target_radius = 1; // 目标半径
    double scale_factor = target_radius / original_radius;
    needed_x_circle *= scale_factor;
    needed_y_circle *= scale_factor;

    /* 首先获取群体机器人的姿态信息 */
    std::vector<std::vector<double> > current_robot_pose(swarm_robot_id.size()); // 存储当前机器人姿态信息的向量

    swarm_robot.getRobotPose(current_robot_pose); // 获取机器人姿态信息
    for(int i = 0; i < swarm_robot_id.size(); i++) {
        cur_x(i) = current_robot_pose[i][0]; // 提取位置信息
        cur_y(i) = current_robot_pose[i][1]; // 提取位置信息
        cur_theta(i) = current_robot_pose[i][2]; // 提取角度信息
    }

    // Debugging
    // double v, w;
    // swarm_robot.calculateVelocity(1, 10, 5, v, w);
    // printf("v: %f, w: %f\n", v, w);


    /* 收敛标志 */
    bool is_conv = false; // 角度收敛标志

    start = clock();

    Eigen::VectorXd needed_x = needed_x_circle;
    Eigen::VectorXd needed_y = needed_y_circle;

    while(! is_conv) { // 当未达到收敛条件时执行以下代码

        /* 判断是否达到收敛条件 */
        del_x = -lap * (cur_x + needed_x); // 计算需要的x的变化
        del_y = -lap * (cur_y + needed_y); // 计算需要的y的变化
        is_conv = true; // 假设已经达到收敛条件
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            if ( (std::fabs(del_x(i)) > conv_x) or (std::fabs(del_y(i)) > conv_y) ) {
                is_conv = false; // 如果任何一个x坐标的变化大于阈值，则认为未收敛
            }       
        }

        // 先转一下
        for (int i = 0; i < swarm_robot_id.size(); i++) {
            v_theta(i) = std::atan2(del_y(i) , del_x(i));
            del_theta(i) = -(cur_theta(i) - v_theta(i));
            while (del_theta(i) < -pi or del_theta(i) > pi) {
                if (del_theta(i) < -pi) del_theta(i) += 2 * pi;
                if (del_theta(i) > pi) del_theta(i) -= 2 * pi;
            }           
        }
        /* Swarm robot move */
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            if (std::fabs(del_theta(i)) > 0.1) {
                w = del_theta(i) / std::fabs(del_theta(i)) * MAX_W;
                v = 0;
            }
            else {
                w = del_theta(i) / std::fabs(del_theta(i)) * MIN_W;
                v = std::sqrt(std::pow(del_x(i),2) + std::pow(del_y(i),2));
                v = swarm_robot.checkVel(v, MAX_V, MIN_V);
            }    
            swarm_robot.moveRobot(i, v, w);  
        }
        ros::Duration(0.05).sleep();

        // double k_v = 0.1; 已经定义过了
        // del_x *= k_v; // 缩放x的变化
        // del_y *= k_v; // 缩放y的变化
        // swarm_robot.moveRobotsbyU(del_x, del_y); // 移动机器人
        // ros::Duration(0.5).sleep();

        swarm_robot.getRobotPose(current_robot_pose); // 获取机器人姿态信息
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            cur_x(i) = current_robot_pose[i][0]; // 提取位置信息
            cur_y(i) = current_robot_pose[i][1]; // 提取位置信息
            cur_theta(i) = current_robot_pose[i][2]; // 提取角度信息
        }
    }
    // while(1){
    //     /* 获取当前机器人的距离信息 */
    //     swarm_robot.calculate_all_Distance();
    //     swarm_robot.calculate_all_x_distance();
    //     swarm_robot.calculate_all_y_distance();

    //     swarm_robot.HardGraph2Speed(Gx0, Gy0);

    //     for (int i = 0; i < swarm_robot_id.size()-1 ; i++) {
    //         double ux = swarm_robot.ux(i);
    //         double uy = swarm_robot.uy(i);
    //         swarm_robot.moveRobotbyU(i, ux, uy);
    //     }
    //     ros::Duration(0.05).sleep();
    // }
    /* 停止所有机器人的运动 */
    swarm_robot.stopRobot(); // 调用停止机器人运动的方法

    end = clock();
    ROS_INFO_STREAM("Succeed!"); // 输出成功消息
    ROS_INFO_STREAM("Total time " << double(end - start)/CLOCKS_PER_SEC << "S");
    return 0; // 返回0，表示程序正常结束
}
    