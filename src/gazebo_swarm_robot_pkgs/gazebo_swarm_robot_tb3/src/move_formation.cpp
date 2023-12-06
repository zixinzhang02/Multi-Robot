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
    std::vector<int> swarm_robot_id{1, 2, 3, 4, 5, 6}; // 创建包含群体机器人ID的向量

    /* 初始化群体机器人对象 */
    SwarmRobot swarm_robot(&nh, swarm_robot_id); // 创建 SwarmRobot 类的对象，用于控制一组机器人


    /* 设置拉普拉斯矩阵 */
    Eigen::MatrixXd lap(swarm_robot_id.size(), swarm_robot_id.size()); // 创建拉普拉斯矩阵对象
    lap <<  5, -1, -1, -1, -1, -1,
            -1, 5, -1, -1, -1, -1,
            -1, -1, 5, -1, -1, -1,
            -1, -1, -1, 5, -1, -1,
            -1, -1, -1, -1, 5, -1,
            -1, -1, -1, -1, -1, 5;

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

    needed_x_circle << 1.71, 0, -1.71, -1.71, 0, 1.71;
    needed_y_circle << -1, -2, -1, 1, 2, 1;

    needed_x_rectangle << 1.71, 0, -1.71, -1.71, 0, 1.71;
    needed_y_rectangle << -1, -1, -1, 1, 1, 1;

    // 计算needed_x_circle[0] 和 needed_y_circle[0]的距离
    double original_radius = std::sqrt(std::pow(needed_x_circle[0], 2) + std::pow(needed_y_circle[0], 2));
    double target_radius = 1; // 目标半径
    double scale_factor = target_radius / original_radius;
    needed_x_circle *= scale_factor / 2.2;
    needed_y_circle *= scale_factor / 2.2;

    needed_x_rectangle *= scale_factor / 2;
    needed_y_rectangle *= scale_factor / 2;

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

    /* 形成圆形编队 */
    Eigen::VectorXd needed_x = needed_x_circle;
    Eigen::VectorXd needed_y = needed_y_circle;

    while(! is_conv) { // 当未达到收敛条件时执行以下代码

        /* 判断是否达到收敛条件 */
        del_x = -lap * (cur_x + needed_x); // 计算需要的x的变化
        del_y = -lap * (cur_y + needed_y); // 计算需要的y的变化
        is_conv = true; // 假设已经达到收敛条件
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            cout << del_x(i) << " " << del_y(i) << endl;
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

    cout << "Suceessfully form a circle" << endl;
    swarm_robot.stopRobot();
    ros::Duration(2).sleep();

     /* 形成长方形编队 */
    needed_x = needed_x_rectangle;
    needed_y = needed_y_rectangle;

    is_conv = false; 
    
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

    cout << "Suceessfully form a rectangle" << endl;
    swarm_robot.stopRobot();
    ros::Duration(2).sleep();


    /* 角度一致性协议 */
    is_conv = false;

    // 先转第0个机器人到目标角度
    double aimed_theta = 0;
    while (!is_conv){
        is_conv = true; 
        swarm_robot.getRobotPose(current_robot_pose);
        for (int i = 0; i < swarm_robot_id.size(); i++) {
            cur_theta(i) = current_robot_pose[i][2];
            double del_angle = -(cur_theta(i) - aimed_theta);
            if(std::fabs(del_angle) > 0.01) {
                is_conv = false; // 如果任何一个角度的变化大于阈值，则认为未收敛
            }      
            w = del_angle / std::fabs(del_angle) * MAX_W * 1;
            swarm_robot.moveRobot(i, 0, w); 
        }
        // del_theta(i) / std::fabs(del_theta(i)) * MAX_W;
        ros::Duration(0.05).sleep();
    }

    cout << "Suceessfully Jiao Du yizhixing" << endl;
    swarm_robot.stopRobot();
    ros::Duration(2).sleep();

    is_conv = false;

    // while(! is_conv) { // 当未达到收敛条件时执行以下代码

    //     /* 判断是否达到收敛条件 */
    //     del_theta = -lap * cur_theta; // 计算角度的变化, Laplace矩阵乘原始角度
    //     del_theta(0) = 0;
    //     is_conv = true; // 假设已经达到收敛条件
    //     for(int i = 0; i < swarm_robot_id.size(); i++) {
    //         if(std::fabs(del_theta(i)) > conv_th) {
    //             is_conv = false; // 如果任何一个角度的变化大于阈值，则认为未收敛
    //         }       
    //     }

    //     /* 移动群体机器人 */
    //     for(int i = 1; i < swarm_robot_id.size(); i++) {
    //         double w = del_theta(i) * k_w; // 计算角速度
    //         w = swarm_robot.checkVel(w, MAX_W, MIN_W); // 限制角速度的范围
    //         swarm_robot.moveRobot(i, 0.0, w); // 控制机器人的运动，只控制角速度
    //     }

    //     /* 等待一段时间以进行机器人移动 */
    //     ros::Duration(0.05).sleep(); // 暂停程序执行0.05秒，等待机器人移动

    //     /* 获取群体机器人的姿态信息 */
    //     swarm_robot.getRobotPose(current_robot_pose); // 获取机器人姿态信息
        
    //     for(int i = 0; i < swarm_robot_id.size(); i++) {
    //         cur_theta(i) = current_robot_pose[i][2]; // 更新角度信息
    //     }
    // }


    swarm_robot.stopRobot(); // 调用停止机器人运动的方法

    end = clock();
    ROS_INFO_STREAM("Succeed!"); // 输出成功消息
    ROS_INFO_STREAM("Total time " << double(end - start)/CLOCKS_PER_SEC << "S");
    return 0; // 返回0，表示程序正常结束
}
    