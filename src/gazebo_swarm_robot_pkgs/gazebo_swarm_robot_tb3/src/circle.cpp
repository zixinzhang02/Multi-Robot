/* 
 * Date: 2021-11-29
 * Description: to the same angle
*/

#include <swarm_robot_control.h> // 包含自定义的头文件
#include <math.h>
#include <time.h>

#define PI acos(-1)

double angle(double x1, double y1, double x2, double y2) {
    double angle_temp;
    double xx, yy;
    xx = x2 - x1;
    yy = y2 - y1;
    if (xx == 0.0) {
        angle_temp = PI / 2.0;
    } else {
        angle_temp = atan(fabs(yy / xx));
    }
    if ((xx < 0.0) && (yy >= 0.0)) {
        angle_temp = PI - angle_temp;
    } else if ((xx < 0.0) && (yy < 0.0)) {
        angle_temp = PI + angle_temp;
    } else if ((xx >= 0.0) && (yy < 0.0)) {
        angle_temp = PI * 2.0 - angle_temp;
    }
    return (angle_temp);
}

void check_angle(double &angle){
    if (angle >= 2*PI)
        angle -= 2*PI;
}

bool Avoid_Crash(std::vector<std::vector<double>> current_robot_pose, int cur_index, int other_index, double & del_theta) {
    std::vector<double> cur_pose = current_robot_pose[cur_index];
    std::vector<double> other_pose = current_robot_pose[other_index];
    double cur_x = cur_pose[0];
    double cur_y = cur_pose[1];
    double other_x = other_pose[0];
    double other_y = other_pose[1];
    // ROS_INFO_STREAM("Calculating coordinates: " << cur_x << cur_y << other_x << other_y);
    double distance = pow((cur_x - other_x), 2) + pow((cur_y - other_y), 2);
    distance = sqrt(distance);
    del_theta = angle(cur_x, cur_y, other_x, other_y);
    double threshold = 0.25;
    if (distance < threshold) {
        ROS_INFO_STREAM("May crash, Distance is " << distance);
        return true;
    }
    else return false;
}


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
    // lap <<  1, -1,  0,  0,  0,
    //         -1, 3, -1,  0, -1,
    //         0, -1,  3, -1, -1,
    //         0,  0, -1,  2, -1,
    //         0, -1, -1, -1,  3; 

    /* 收敛阈值 */
    double conv_x = 0.05;  // x的阈值，单位m

    /* 速度比例和阈值 */
    double MAX_W = 1;       // 最大角速度（弧度/秒）
    double MIN_W = 0.05;    // 最小角速度（弧度/秒）
    double MAX_V = 0.2;     // 最大线性速度（米/秒）
    double MIN_V = 0.01;    // 最小线性速度（米/秒）
    double k_w = 0.1;       // 角速度的缩放比例
    double k_v = 0.1;       // 线性速度的缩放比例

    /* 移动机器人的姿态和下一个姿态 */
    Eigen::VectorXd cur_x(swarm_robot_id.size()); // 当前x坐标
    Eigen::VectorXd cur_y(swarm_robot_id.size()); // 当前y坐标
    Eigen::VectorXd cur_theta(swarm_robot_id.size()); // 当前角度
    Eigen::VectorXd del_x(swarm_robot_id.size()); // x坐标的变化
    Eigen::VectorXd del_y(swarm_robot_id.size()); // y坐标的变化
    Eigen::VectorXd del_theta(swarm_robot_id.size()); // 角度的变化

    /* 首先获取群体机器人的姿态信息 */
    std::vector<std::vector<double> > current_robot_pose(swarm_robot_id.size()); // 存储当前机器人姿态信息的向量

    swarm_robot.getRobotPose(current_robot_pose); // 获取机器人姿态信息

    for(int i = 0; i < swarm_robot_id.size(); i++) {
        cur_theta(i) = current_robot_pose[i][2]; // 提取角度信息
    }

    /* 先随机移动一些角度，防止碰撞 */
    // int initialize_step = 5000;
    // for(int j = 0; j < initialize_step; j++) {
    //     for(int i = 0; i < swarm_robot_id.size(); i++) {
    //         swarm_robot.RandomInitialize(i);
    //         swarm_robot.getRobotPose(current_robot_pose); // 获取机器人姿态信息
    //         cur_theta(i) = current_robot_pose[i][2]; // 更新角度信息
    //         cur_x(i) = current_robot_pose[i][0]; // 更新x坐标信息
    //         cur_y(i) = current_robot_pose[i][1]; // 更新y坐标信息
    //     }
    // }
    // swarm_robot.stopRobot(); // 调用停止机器人运动的方法

    /* 收敛标志 */
    bool is_conv = false; // 速度收敛标志

    /* While 循环 */
    while(! is_conv) { // 当未达到收敛条件时执行以下代码

        /* 判断是否达到收敛条件 */
        del_x = -lap * cur_x; // 计算需要的x的变化
        is_conv = true; // 假设已经达到收敛条件
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            if(std::fabs(del_x(i)) > conv_x) {
                is_conv = false; // 如果任何一个x坐标的变化大于阈值，则认为未收敛
            }       
        }

        /* 移动群体机器人的角度 */
        // for(int i = 0; i < swarm_robot_id.size(); i++) {
        //     double w = del_theta(i) * k_w; // 计算角速度
        //     w = swarm_robot.checkVel(w, MAX_W, MIN_W); // 限制角速度的范围
        //     swarm_robot.moveRobot(i, 0.0, w); // 控制机器人的运动，只控制角速度
        // }

        /* 设置移动群体机器人的速度(并施加避障功能) */
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            double v = del_x(i)/cos(cur_theta(i)) * k_v; // 根所需要调整的x的值（由一致性协议计算得到），计算线速度
            double w = 0;
            v = swarm_robot.checkVel(v, MAX_V, MIN_V); 

            for (int j = 0; j < swarm_robot_id.size(); j++){ // 检查附近是否有其他机器人，如果有，放慢速度并且向反方向转动(角度越小，变化越大)
                if (j == i) continue;
                double del_theta;
                bool may_crash = Avoid_Crash(current_robot_pose, i, j, del_theta); // 判断是否可能发生碰撞，并记录两者之间的方位
                double del_w = 0;
                if (may_crash == true){
                    v = swarm_robot.checkVel(v, MAX_V, MIN_V) / 5;
                    double cur_yaw = cur_theta[i];
                    if (cur_yaw < 0) cur_yaw += 2*PI; // 将ROS输出的方位从(-pi, pi) 转换成(0, 2pi)
                    if (v < 0){ // 速度小于0时，计算要反向
                        cur_yaw += PI;
                        check_angle(cur_yaw);
                    }
                    if ((cur_yaw - del_theta) == 0){
                        del_w = 0.02;
                    }
                    else del_w = 0.1 / (cur_yaw - del_theta);
                }
                w += del_w;
            }
            w = swarm_robot.checkVel(w, MAX_W, MIN_W); // 限制线速度和角速度的范围
            swarm_robot.moveRobot(i, v, w); // 控制机器人的运动，同时控制线速度和角速度
        }

        /* 等待一段时间以进行机器人移动 */
        ros::Duration(0.05).sleep(); // 暂停程序执行0.05秒，等待机器人移动

        /* 获取群体机器人的姿态信息 */
        swarm_robot.getRobotPose(current_robot_pose); // 获取机器人姿态信息
        
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            cur_theta(i) = current_robot_pose[i][2]; // 更新角度信息
            cur_x(i) = current_robot_pose[i][0]; // 更新x坐标信息
            cur_y(i) = current_robot_pose[i][1]; // 更新y坐标信息
        }
    }

    /* 停止所有机器人的运动 */
    swarm_robot.stopRobot(); // 调用停止机器人运动的方法

    end = clock();
    ROS_INFO_STREAM("Succeed!"); // 输出成功消息
    ROS_INFO_STREAM("Total time " << end - start << "ms");
    return 0; // 返回0，表示程序正常结束
}