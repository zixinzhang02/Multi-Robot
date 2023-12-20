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
    /* 收敛阈值 */
    double conv_th = 0.05;  // 角度的阈值，单位弧度
    double conv_x = 0.1;  // x的阈值，单位m
    double conv_y = 0.1;  // y的阈值，单位m

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

    Eigen::VectorXd needed_x_rectangle(swarm_robot_id.size());
    Eigen::VectorXd needed_y_rectangle(swarm_robot_id.size());

    needed_x_cross << 1, 0, 0, 0, -1;
    needed_y_cross << 0, -1, 0, 1, 0;

    Eigen::VectorXd needed_x = needed_x_cross;
    Eigen::VectorXd needed_y = needed_y_cross;

    // Eigen::MatrixXd Gd_hexagon << 0, 1, 1, 1, 1, 1, 1

    start = clock();

    conv_x = 0.08;
    conv_y = 0.08;
    swarm_robot.Formation(needed_x, needed_y, lap, conv_x, conv_y);
    swarm_robot.ChangeFormationDirection(pi/2);

    double t = 0;
    while (t < 3000){
        // swarm_robot.MoveFormation(needed_x, needed_y, lap, 0.2, 2);
        t += 60;
    }
   
    // swarm_robot.ChangeFormationDirection(0);
    // swarm_robot.MoveFormation(needed_x_circle, needed_y_circle, lap, 0.2, 5);

    // conv_x = 0.05;
    // conv_y = 0.05;
    // swarm_robot.Formation(needed_x_rectangle, needed_y_rectangle, lap, conv_x, conv_y);

    // conv_x = 0.05;
    // conv_y = 0.05;
    // swarm_robot.Formation(needed_x_circle, needed_y_circle, lap, conv_x, conv_y);
    

    end = clock();
    ROS_INFO_STREAM("Succeed!"); // 输出成功消息
    ROS_INFO_STREAM("Total time " << double(end - start)/CLOCKS_PER_SEC << "S");
    return 0; // 返回0，表示程序正常结束
}
    