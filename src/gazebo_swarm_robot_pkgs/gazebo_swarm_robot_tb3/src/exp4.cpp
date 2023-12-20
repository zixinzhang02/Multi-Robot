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
    std::vector<int> swarm_robot_id{1, 2, 3, 4, 5, 6, 7}; // 创建包含群体机器人ID的向量

    /* 初始化群体机器人对象 */
    SwarmRobot swarm_robot(&nh, swarm_robot_id); // 创建 SwarmRobot 类的对象，用于控制一组机器人

    /* 设置拉普拉斯矩阵 */
    Eigen::MatrixXd lap(swarm_robot_id.size(), swarm_robot_id.size()); // 创建拉普拉斯矩阵对象
    lap <<  6, -1, -1, -1, -1, -1, -1,
            -1, 6, -1, -1, -1, -1, -1,
            -1, -1, 6, -1, -1, -1, -1,
            -1, -1, -1, 6, -1, -1, -1,
            -1, -1, -1, -1, 6, -1, -1,
            -1, -1, -1, -1, -1, 6, -1,
            -1, -1, -1, -1, -1, -1, 6;
    /* 收敛阈值 */
    double conv_th = 0.05;  // 角度的阈值，单位弧度
    double conv_x = 0.1;  // x的阈值，单位m
    double conv_y = 0.1;  // y的阈值，单位m

    /* 速度比例和阈值 */
    // double MAX_W = 1;       // 最大角速度（弧度/秒）
    // double MIN_W = 0.05;    // 最小角速度（弧度/秒）
    // double MAX_V = 0.2;     // 最大线性速度（米/秒）
    // double MIN_V = 0.01;    // 最小线性速度（米/秒）
    // double k_w = 0.1;       // 角速度的缩放比例
    // double k_v = 0.1;       // 线性速度的缩放比例

    double v,w; // 机器人的线速度和角速度

    /*编队*/
    Eigen::MatrixXd Gd_hexagon(swarm_robot_id.size(), swarm_robot_id.size()); 
    Eigen::MatrixXd Gd_gong(swarm_robot_id.size(), swarm_robot_id.size());
    Gd_hexagon << 0, 1, 1, 1, 1, 1, 1, // 半径为1
                1, 0, 2, 1.732, 1, 1, 1.732,
                1, 2, 0, 1, 1.732, 1.732, 1,
                1, 1.732, 1, 0, 1, 2, 1.732,
                1, 1, 1.732, 1, 0, 1.732, 2,
                1, 1, 1.732, 2, 1.732, 0, 1,
                1, 1.732, 1, 1.732, 2, 1, 0;
    // 交换Gd_hexagon的第3行和第4行
    // Eigen::MatrixXd temp = Gd_hexagon.row(2);
    // Gd_hexagon.row(2) = Gd_hexagon.row(4);
    // Gd_hexagon.row(4) = temp;
    // // 交换Gd_hexagon的第3列和第4列
    // temp = Gd_hexagon.col(2);
    // Gd_hexagon.col(2) = Gd_hexagon.col(4);
    // Gd_hexagon.col(4) = temp;

    Gd_gong << 0, 1.414, 1.414, 1, 1.414, 1, 1.414, // 边长为1
                1.414, 0, 2.828, 2.236, 2, 1, 2,
                1.414, 2.828, 0, 1, 2, 2.236, 2,
                1, 2.236, 1, 0, 1, 2, 2.236,
                1.414, 2, 2, 1, 0, 2.236, 2.828,
                1, 1, 2.236, 2, 2.236, 0, 1,
                1.414, 2, 2, 2.236, 2.828, 1, 0;

    Eigen::VectorXd needed_x_hexagon(swarm_robot_id.size()); 
    Eigen::VectorXd needed_y_hexagon(swarm_robot_id.size());
    needed_x_hexagon << 0, -1, 1, 0.5, -0.5, -0.5, 0.5;
    needed_y_hexagon << 0, 0, 0, 0.866, 0.866, -0.866, -0.866;

    Eigen::VectorXd needed_x_gong(swarm_robot_id.size());
    Eigen::VectorXd needed_y_gong(swarm_robot_id.size());
    needed_x_gong << 0, -1, 1, 0, -1, 0, 1;
    needed_y_gong << 0, -1, 1, 1, 1, -1, -1;

    double scale_factor = 0.5;
    // needed_x_hexagon = needed_x_hexagon * scale_factor;
    // needed_y_hexagon = needed_y_hexagon * scale_factor;
    // needed_x_gong = needed_x_gong * scale_factor;
    // needed_y_gong = needed_y_gong * scale_factor;

    // // 交换needed_x_hexagon的第3行和第4行
    // double temp_x = needed_x_hexagon(2);
    // needed_x_hexagon(2) = needed_x_hexagon(4);
    // needed_x_hexagon(4) = temp_x;
    // // 交换needed_y_hexagon的第3个和第4个
    // double temp_y = needed_y_hexagon(2);
    // needed_y_hexagon(2) = needed_y_hexagon(4);
    // needed_y_hexagon(4) = temp_y;

    Eigen::VectorXd needed_x = needed_x_hexagon;
    Eigen::VectorXd needed_y = needed_y_hexagon;

    needed_x = needed_x_gong;
    needed_y = needed_y_gong;

    start = clock();

    conv_x = 0.08;
    conv_y = 0.08;
    swarm_robot.Formation(needed_x, needed_y, lap, conv_x, conv_y);

    needed_x *= scale_factor;
    needed_y *= scale_factor;

    swarm_robot.Formation(needed_x, needed_y, lap, conv_x, conv_y);

    // swarm_robot.ChangeFormationDirection(pi/2);
    double t = 0;
    while (t < 10000){
        Eigen::MatrixXd Gd = Gd_gong;
        swarm_robot.MoveFormation(Gd, lap, 0, 0.1);
        t += 50;
    }
    swarm_robot.stopRobot();
    // swarm_robot.ChangeFormationDirection(0);

    t = 0;
    while (t < 10000){
        Eigen::MatrixXd Gd = Gd_gong;
        swarm_robot.MoveFormation(Gd, lap, 0.1, 0);
        t += 50;
    }

    swarm_robot.stopRobot();
   
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
    