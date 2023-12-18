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
#include <Eigen/Core>
#define pi acos(-1)

/* 主函数 */
int main(int argc, char** argv) {
    clock_t start, end;
    start = clock();
    ros::init(argc, argv, "swarm_robot_control_formation"); // 初始化ROS节点，节点名称为 "swarm_robot_control_formation"
    ros::NodeHandle nh; // 创建ROS节点句柄

    /* 第一步：基于Aruco标记设置群体机器人的ID */
    std::vector<int> swarm_robot_id{1, 2, 3, 4, 5};
    std::vector<int> dynamic_robot_id{0, 1, 2, 3, 4};  // 创建包含群体机器人ID的向量

    /* 初始化群体机器人对象 */
    SwarmRobot swarm_robot(&nh, swarm_robot_id); // 创建 SwarmRobot 类的对象，用于控制一组机器人
    /* 设置拉普拉斯矩阵 */
    Eigen::MatrixXd lap(swarm_robot_id.size(), swarm_robot_id.size()); // 创建拉普拉斯矩阵对象
    lap <<  4, -1, -1, -1, -1,
            -1, 4, -1, -1, -1,
            -1, -1, 4, -1, -1,
            -1, -1, -1, 4, -1,
            -1, -1, -1, -1, 4;

    /*编队*/
    Eigen::VectorXd needed_x_circle(swarm_robot_id.size());
    Eigen::VectorXd needed_y_circle(swarm_robot_id.size());
    needed_x_circle << 2, 0, -2, -1, 1;
    needed_y_circle << -1, -2.5, -1, 1, 1;
    // 计算needed_x_circle[0] 和 needed_y_circle[0]的距离
    double original_radius = std::sqrt(std::pow(needed_x_circle[0], 2) + std::pow(needed_y_circle[0], 2));
    double target_radius = 1; // 目标半径
    double scale_factor = target_radius / original_radius;
    needed_x_circle *= scale_factor;
    needed_y_circle *= scale_factor;

    double conv_x = 0.05;
    double conv_y = 0.05;

    start = clock();

    swarm_robot.Formation(needed_x_circle, needed_y_circle, lap, conv_x, conv_y);

    end = clock();
    ROS_INFO_STREAM("Succeed!"); // 输出成功消息
    ROS_INFO_STREAM("Total time " << double(end - start)/CLOCKS_PER_SEC << "S");
    return 0; // 返回0，表示程序正常结束
}
    