/*
 * Date: 2021-11-29
 * Description: the basic function
 */

#include <swarm_robot_control.h>

SwarmRobot::SwarmRobot(ros::NodeHandle *nh, std::vector<int> swarm_robot_id_) : swarm_robot_id(swarm_robot_id_)
{

    this->robot_num = swarm_robot_id.size();

    std::cout << "robot_num=" << robot_num << std::endl;
    /* Initialize swarm robot */
    for (int i = 0; i < 10; i++)
    {
        std::string vel_topic = "/robot_" + std::to_string(i + 1) + "/cmd_vel";
        cmd_vel_pub[i] = nh_.advertise<geometry_msgs::Twist>(vel_topic, 10);
    }
}

SwarmRobot::~SwarmRobot()
{
}

/* Get the [index]th robot's pose to vector*/
bool SwarmRobot::getRobotPose(int index, std::vector<double> &pose_cur)
{

    pose_cur.resize(3, 0.0);
    tf::StampedTransform transform;
    std::string robot_frame = "robot_" + std::to_string(this->swarm_robot_id[index]) + "/base_footprint";
    std::string base_marker = "robot_" + std::to_string(this->swarm_robot_id[index]) + "/odom";

    // Try to get pose of robot from tf
    try
    {
        this->tf_listener.waitForTransform(base_marker, robot_frame, ros::Time(0), ros::Duration(0.5));
        this->tf_listener.lookupTransform(base_marker, robot_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        // ros::Duration(1.0).sleep();
        return false;
    }

    tf::Quaternion q = transform.getRotation();
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Get pose
    pose_cur.resize(3);
    pose_cur[0] = transform.getOrigin().x();
    pose_cur[1] = transform.getOrigin().y();
    pose_cur[2] = yaw;

    ROS_INFO_STREAM("Get pose of robot_" << swarm_robot_id[index] << " is: x=" << pose_cur[0] << " y=" << pose_cur[1] << " theta=" << pose_cur[2]);
    return true;
}

/* Get gazebo pose of swarm robot */
bool SwarmRobot::getRobotPose(std::vector<std::vector<double>> &current_robot_pose)
{

    current_robot_pose.resize(this->robot_num);
    std::vector<bool> flag_pose(this->robot_num, false);

    bool flag = false;

    while (!flag)
    {
        flag = true;
        for (int i = 0; i < this->robot_num; i++)
        {
            flag = flag && flag_pose[i];
        }
        for (int i = 0; i < this->robot_num; i++)
        {
            std::vector<double> pose_robot(3);
            if (getRobotPose(i, pose_robot))
            {
                current_robot_pose[i] = pose_robot;
                flag_pose[i] = true;
            }
        }
    }
    ROS_INFO_STREAM("Succeed getting pose!");
    return true;
}

/* Move robot: index represents the order in swarm_robot_id*/
bool SwarmRobot::moveRobot(int index, double v, double w)
{
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = v;
    vel_msg.angular.z = w;
    cmd_vel_pub[swarm_robot_id[index] - 1].publish(vel_msg);
    ROS_INFO_STREAM("Move robot_" << swarm_robot_id[index] << " with v=" << v << " w=" << w);
    return true;
}

/* Move all robots*/
bool SwarmRobot::moveRobot(std::vector<std::vector<double>> &speed)
{
    if (this->robot_num != speed.size())
    {
        ROS_INFO_STREAM("The robot number does not equal the speed number!");
        return false;
    }

    for (int i = 0; i < this->robot_num; i++)
    {
        if (!this->moveRobot(this->swarm_robot_id[i], speed[i][0], speed[i][1]))
        {
            return false;
        }
    }
    return true;
}

/* Stop robot */
bool SwarmRobot::stopRobot(int index)
{
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0.0;
    vel_msg.angular.z = 0.0;
    cmd_vel_pub[swarm_robot_id[index] - 1].publish(vel_msg);
    ROS_INFO_STREAM("Stop robot_" << swarm_robot_id[swarm_robot_id[index]]);
    return true;
}

/* Stop all robot */
bool SwarmRobot::stopRobot()
{

    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0.0;
    vel_msg.angular.z = 0.0;
    for (int i = 0; i < this->robot_num; i++)
    {
        cmd_vel_pub[this->swarm_robot_id[i] - 1].publish(vel_msg);
    }
    ROS_INFO_STREAM("Stop all robots.");
    return true;
}

/* Check velocity */
double SwarmRobot::checkVel(double v, double max_v, double min_v)
{
    if (max_v <= 0 || min_v <= 0)
    {
        std::cout << "Error input of checkW()" << std::endl;
        return v;
    }

    if (v > 0)
    {
        v = std::max(v, min_v);
        v = std::min(v, max_v);
    }
    else
    {
        v = std::min(v, -min_v);
        v = std::max(v, -max_v);
    }
    return v;
}

/*角速度线速度转化为两个方向上的速度*/
void SwarmRobot::U2VW(int index, double ux, double uy, double &v, double &w)
{
    v = 0;
    w = 0;
    double pi = 3.141592653589793;
    std::vector<double> pose_cur;
    getRobotPose(index, pose_cur);
    double x_robot = pose_cur[0];
    double y_robot = pose_cur[1];
    double theta_robot = pose_cur[2];
    double v0 = std::sqrt(ux * ux + uy * uy);
    double theta_v = 0;
    double theta_v_sin = 0;
    double theta_v_cos = 0;
    /*确定速度方向*/
    theta_v_cos = std::acos(ux / std::sqrt(ux * ux + uy * uy));
    theta_v_sin = std::asin(uy / std::sqrt(ux * ux + uy * uy));
    if (uy > 0)
    {
        theta_v = theta_v_cos;
    }
    else if (uy < 0)
    {
        theta_v = -theta_v_cos;
    }
    else
    {
        if (ux == 0)
        {
            theta_v = theta_robot;
        }
        else
        {
            if (uy > 0)
            {
                theta_v = pi / 2;
            }
            else if (uy < 0)
            {
                theta_v = -pi / 2;
            }
        }
    }
    /*限定angle大小*/
    double angle = theta_v - theta_robot;
    while (angle > pi || angle < -pi)
    {
        if (angle > pi)
        {
            angle = angle - 2 * pi;
        }
        else
        {
            angle = angle + 2 * pi;
        }
    }

    double W = 1; // 角速度最大值参考参数
    double V = 1; // 速度方向
    /*速度控制*/
    if (angle > pi / 2)
    {
        angle = angle - pi;
        // 速度反向
        V = -1;
    }
    else if (angle < -pi / 2)
    {
        angle = pi - angle;
        // 速度反向
        V = -1;
    }
    else
    {
        // 速度正向
        V = 1;
    }
    w = W * (angle / std::abs(angle)) * (std::exp(std::abs(angle)) - 1);
    v = V * v0 * std::exp(-std::abs(angle));
    if (v > 0 && v > 0.5)
    {
        v = 0.5;
    }
    else if (v < 0 && v < -0.5)
    {
        v = -0.5;
    }
    else if (v == 0)
    {
        w = 0;
    }
    
    /*deug*/
    std::cout << "//angle = " << angle << endl;
    std::cout << "//theta_v = " << theta_v << endl;
    std::cout << "//theta_robot = " << theta_robot << endl;
    std::cout << "//v = " << v << endl;
    std::cout << "//w = " << w << endl;
}

/*按照两个分速度移动机器人*/
void SwarmRobot::MoveRobot(int index, double ux, double uy)
{
    double v = 0;
    double w = 0;
    U2VW(index, ux, uy, v, w);
    moveRobot(index, v, w);
}

/*让机器人去到一个固定位置*/
void SwarmRobot::ComeDot(int index, double x0, double y0, double &ux, double &uy)
{
    double pi = 3.141592653589793;
    double T = 50;
    std::vector<double> pose_cur;
    getRobotPose(index, pose_cur);
    double x_robot = pose_cur[0];
    double y_robot = pose_cur[1];
    double theta_robot = pose_cur[2];

    // ux = (x0 - x_robot)*(x0 - x_robot)*(x0 - x_robot) / T;
    // uy = (y0 - y_robot)* (y0 - y_robot)* (y0 - y_robot)/ T;
    ux = (x0 - x_robot) / T;
    uy = (y0 - y_robot) / T;
}

/*获得机器人相对位置矩阵*/
void SwarmRobot::GetGxGyGd(Eigen::MatrixXd &Gx, Eigen::MatrixXd &Gy, Eigen::MatrixXd &Gd)
{
    std::vector<double> pose_curi;
    std::vector<double> pose_curj;

    for (int i = 0; i < swarm_robot_id.size(); i++)
    {
        getRobotPose(i, pose_curi);
        for (int j = 0; j < swarm_robot_id.size(); j++)
        {
            getRobotPose(j, pose_curj);
            Gx(i, j) = pose_curi[0] - pose_curj[0];
            Gy(i, j) = pose_curi[1] - pose_curj[1];
            Gd(i, j) = std::sqrt(Gx(i, j) * Gx(i, j) + Gy(i, j) * Gy(i, j));
        }
    }
    /*dubug*/
    std::cout << "Gx =" << endl;
    std::cout << Gx << endl;
    std::cout << "********************************************************" << endl;
    std::cout << "Gy=" << endl;
    std::cout << Gy << endl;
    std::cout << "********************************************************" << endl;
    std::cout << "Gd=" << endl;
    std::cout << Gd << endl;
    std::cout << "********************************************************" << endl;
}

/*通过相对位置计算速度*/
void SwarmRobot::GetVelocity(Eigen::MatrixXd Gd0, Eigen::MatrixXd Gd, double *ux, double *uy)
{
    std::vector<double> pose_curi;
    std::vector<double> pose_curj;
    double d = 0;
    Eigen::MatrixXd gd(swarm_robot_id.size(), swarm_robot_id.size());
    gd = (Gd - Gd0);
    std::cout << "gd =" << endl;
    std::cout << gd << endl;
    std::cout << "********************************************************" << endl;
    std::cout << "Gd0=" << endl;
    std::cout << Gd0 << endl;
    std::cout << "********************************************************" << endl;
    std::cout << "Gd=" << endl;
    std::cout << Gd << endl;
    std::cout << "********************************************************" << endl;

    /*获得方向向量,一号车不动,二号车与一号车距离固定，其它车以这两两车为目标计算速度*/
    ux[0] = 0;
    uy[0] = 0;
    double direction[2] = {0};
    double direction0[2] = {0};
    double direction1[2] = {0};
    /*初始化速度*/
    for (int i = 0; i < swarm_robot_id.size(); i++)
    {
        std::cout << "ux" << i << "=" << ux[i] << endl;
        std::cout << "uy" << i << "=" << uy[i] << endl;
        ux[i] = 0;
        uy[i] = 0;
    }
    /*获得车的速度*/
    /*二号车速度计算*/
    for (int i = 1; i < 2; i++)
    {
        getRobotPose(i, pose_curi);
        for (int j = 0; j < 2; j++)
        {
            getRobotPose(j, pose_curj);
            /*计算面对车1（或2）的方向向量*/
            direction[0] = pose_curj[0] - pose_curi[0];
            direction[1] = pose_curj[1] - pose_curi[1];
            d = std::sqrt(direction[0] * direction[0] + direction[1] * direction[1]);
            if (d != 0)
            {
                direction0[0] = direction[0] / d;
                direction0[1] = direction[1] / d;
            }
            else
            {
                direction0[0] = 0;
                direction0[1] = 0;
            }
            ux[i] += gd(i, j) * direction0[0];
            uy[i] += gd(i, j) * direction0[1];
        }
    }
    /*3和之后的车的速度计算*/
    for (int i = 2; i < swarm_robot_id.size(); i++)
    {
        getRobotPose(i, pose_curi);
        for (int j = 0; j < 3; j++)
        {
            getRobotPose(j, pose_curj);
            /*计算面对车1（或2）的方向向量*/
            direction[0] = pose_curj[0] - pose_curi[0];
            direction[1] = pose_curj[1] - pose_curi[1];
            d = std::sqrt(direction[0] * direction[0] + direction[1] * direction[1]);
            if (d != 0)
            {
                direction0[0] = direction[0] / d;
                direction0[1] = direction[1] / d;
            }
            else
            {
                direction0[0] = 0;
                direction0[1] = 0;
            }
            ux[i] += gd(i, j) * direction0[0];
            uy[i] += gd(i, j) * direction0[1];
        }
    }
}


    // for (int i = 1; i < swarm_robot_id.size(); i++)
    // {
    //     getRobotPose(i, pose_curi);
    //     for (int j = 0; j < 2; j++)
    //     {
    //         getRobotPose(j, pose_curj);
    //         direction[0] = pose_curj[0] - pose_curi[0];
    //         direction[1] = pose_curj[1] - pose_curi[1];

    //         d = std::sqrt(direction[0] * direction[0] + direction[1] * direction[1]);
    //         if (d != 0)
    //         {
    //             direction[0] = direction[0] / d;
    //             direction[1] = direction[1] / d;
    //         }
    //         ux[i] = ux[i] + direction[0] * gd(i, j);
    //         uy[i] = uy[i] + direction[1] * gd(i, j);
    //     }
    // }