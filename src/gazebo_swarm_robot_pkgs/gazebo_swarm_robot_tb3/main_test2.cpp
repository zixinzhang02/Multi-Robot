/*
 * Date: 2021-11-29
 * Description: to the same angle
 * 红色是x轴
 */

#include <swarm_robot_control.h>

/* Main function */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "swarm_robot_control_formation");
    ros::NodeHandle nh;

    /* First: Set ids of swarm robot based on Aruco marker */
    std::vector<int> swarm_robot_id{1, 2, 3, 4, 5};

    /* Initialize swarm robot */
    SwarmRobot swarm_robot(&nh, swarm_robot_id);

    /* Set L Matrix*/
    Eigen::MatrixXd lap(swarm_robot_id.size(), swarm_robot_id.size());
    lap << 4, -1, -1, -1, -1,
        -1, 4, -1, -1, -1,
        -1, -1, 4, -1, -1,
        -1, -1, -1, 4, -1,
        -1, -1, -1, -1, 4;

    /* Convergence threshold */
    double conv_th = 0.05; // Threshold of angle, in rad

    /* Velocity scale and threshold */
    double MAX_W = 1;    // Maximum angle velocity (rad/s)
    double MIN_W = 0.05; // Minimum angle velocity(rad/s)
    double MAX_V = 0.2;  // Maximum linear velocity(m/s)
    double MIN_V = 0.01; // Minimum linear velocity(m/s)
    double k_w = 0.1;    // Scale of angle velocity
    double k_v = 0.1;    // Scale of linear velocity

    /* Mobile robot poses and for next poses */
    Eigen::VectorXd cur_x(swarm_robot_id.size());
    Eigen::VectorXd cur_y(swarm_robot_id.size());
    Eigen::VectorXd cur_theta(swarm_robot_id.size());
    Eigen::VectorXd del_x(swarm_robot_id.size());
    Eigen::VectorXd del_y(swarm_robot_id.size());
    Eigen::VectorXd del_theta(swarm_robot_id.size());

    /* Get swarm robot poses firstly */
    std::vector<std::vector<double>> current_robot_pose(swarm_robot_id.size());

    swarm_robot.getRobotPose(current_robot_pose);
    /*initialize the G*/
    Eigen::MatrixXd Gx(swarm_robot_id.size(), swarm_robot_id.size());
    Eigen::MatrixXd Gy(swarm_robot_id.size(), swarm_robot_id.size());
    Eigen::MatrixXd Gd(swarm_robot_id.size(), swarm_robot_id.size());
    /*initialize the g*/
    Eigen::MatrixXd gx(swarm_robot_id.size(), swarm_robot_id.size());
    Eigen::MatrixXd gy(swarm_robot_id.size(), swarm_robot_id.size());

    /*initialize the robot speed*/
    double ux[swarm_robot_id.size()] = {0};
    double uy[swarm_robot_id.size()] = {0};

    /*over sign*/
    bool is_conv = false;
    /*initialize the G0*/
    // Eigen::MatrixXd Gx0(swarm_robot_id.size(), swarm_robot_id.size()); // 创建Gx0矩阵对象
    // Eigen::MatrixXd Gy0(swarm_robot_id.size(), swarm_robot_id.size()); // 创建Gy0矩阵对象
    Eigen::MatrixXd Gd0(swarm_robot_id.size(), swarm_robot_id.size()); // 创建Gy0矩阵对象
    Eigen::MatrixXd gd(swarm_robot_id.size(), swarm_robot_id.size());  // 创建Gy0矩阵对象
    // Gx0 << 0, -1, -1, -1, -2,
    //     1, 0, 0, 0, -1,
    //     1, 0, 0, 0, -1,
    //     1, 0, 0, 0, -1,
    //     2, 1, 1, 1, 0;
    // Gy0 << 0, 1, 0, -1, 0,
    //     -1, 0, -1, -2, -1,
    //     0, 1, 0, -1, 0,
    //     1, 2, 1, 0, 1,
    //     0, 1, 0, -1, 0;
    Gd0 << 0, 1.414, 2, 1.414, 1,
        1.414, 0, 1.414, 2, 1,
        2, 1.414, 0, 1.414, 1,
        1.414, 2, 1.414, 0, 1,
        1, 1, 1, 1, 0;
    gd = Gd0 - Gd;
    /* While loop */
    double iter = 1;
    while (iter!=0)
    {
        /*初始化速度*/
        for (int i = 0; i < swarm_robot_id.size(); i++)
        {
            std::cout << "ux" << i << "=" << ux[i] << endl;
            std::cout << "uy" << i << "=" << uy[i] << endl;
            ux[i] = 0;
            uy[i] = 0;
        }

        /*获得当前距离矩阵*/
        swarm_robot.GetGxGyGd(Gx, Gy, Gd);

        /*计算获得保持队形需要速度*/
        swarm_robot.GetVelocity(Gd0, Gd, ux, uy);

        /*叠加直线运动速度*/
        ux[0] += -0.1;
        uy[0] += 0;
        ux[1] += -0.1;
        uy[1] += 0;

        /*移动全部机器人*/
        for (int i = 0; i < swarm_robot_id.size(); i++)
        {
            swarm_robot.MoveRobot(i, ux[i], uy[i]);
        }

        /*收敛性检测*/
        iter = 1;
        gd = Gd0 - Gd;
        // std::cout << "gd = " <<gd<< endl;
        std::cout << "iter = " <<iter << endl;
        for (int i = 0; i < swarm_robot_id.size(); i++)
        {
            for (int j = 0; j < swarm_robot_id.size(); j++)
            {
                std::cout << "gd(i,j) = " <<gd(i,j) << endl;
                iter += std::abs(gd(i,j));
                std::cout << "iter = " <<iter << endl;
            }
        }
        if(iter < 0.05)
        {
            iter = 0;
        }


        /* 机器人移动时间，即更新频率 */
        ros::Duration(0.05).sleep();
    }

    /* 停止所有机器人 */
    swarm_robot.stopRobot();

    /*成功返回Succeed，0*/
    ROS_INFO_STREAM("Succeed!");
    return 0;
}
