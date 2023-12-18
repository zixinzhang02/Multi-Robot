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

// 动态编队相关
int findClosestRobot(Eigen::VectorXd cur_x, Eigen::VectorXd cur_y) {
    int numRobots = cur_x.size();
    if (numRobots != cur_y.size()) {
        std::cerr << "Vector sizes do not match!" << std::endl;
        return -1; // 返回无效的ID
    }

    // 计算坐标的平均值
    double mean_x = cur_x.mean();
    double mean_y = cur_y.mean();

    // 初始化最小距离和最近机器人的ID
    double minDistance = std::numeric_limits<double>::max();
    int closestRobotID = -1;

    // 计算每个机器人到平均值的距离并找到最近的机器人
    for (int i = 0; i < numRobots; ++i) {
        double distance = std::sqrt(std::pow(cur_x[i] - mean_x, 2) + std::pow(cur_y[i] - mean_y, 2));
        if (distance < minDistance) {
            minDistance = distance;
            closestRobotID = i;
        }
    }

    return closestRobotID;
}

/* 主函数 */
int main(int argc, char** argv) {
    clock_t start, end;
    start = clock();
    ros::init(argc, argv, "swarm_robot_control_formation"); // 初始化ROS节点，节点名称为 "swarm_robot_control_formation"
    ros::NodeHandle nh; // 创建ROS节点句柄

    /* 第一步：基于Aruco标记设置群体机器人的ID */
    std::vector<int> swarm_robot_id{1, 2, 3, 4, 5}; // 创建包含群体机器人ID的向量
    std::vector<int> dynamic_robot_id{0, 1, 2, 3, 4}; //

    /* 初始化群体机器人对象 */
    SwarmRobot swarm_robot(&nh, swarm_robot_id); // 创建 SwarmRobot 类的对象，用于控制一组机器人
    std::cout << "11111111";

    /* 设置拉普拉斯矩阵 */
    Eigen::MatrixXd lap(swarm_robot_id.size(), swarm_robot_id.size()); // 创建拉普拉斯矩阵对象
    lap <<  4, -1, -1, -1, -1,
            -1, 4, -1, -1, -1,
            -1, -1, 4, -1, -1,
            -1, -1, -1, 4, -1,
            -1, -1, -1, -1, 4;
    /*编队*/
    Eigen::VectorXd needed_x_cross(swarm_robot_id.size());
    Eigen::VectorXd needed_y_cross(swarm_robot_id.size());

    needed_x_cross << 1, 0, 0, 0, -1;
    needed_y_cross << 0, -1, 0, 1, 0;

    double conv_x = 0.05;
    double conv_y = 0.05;

    swarm_robot.Formation(needed_x_cross, needed_y_cross, lap, conv_x, conv_y);

    end = clock();
    ROS_INFO_STREAM("Succeed!"); // 输出成功消息
    ROS_INFO_STREAM("Total time " << double(end - start)/CLOCKS_PER_SEC << "S");
    return 0; // 返回0，表示程序正常结束
}
    