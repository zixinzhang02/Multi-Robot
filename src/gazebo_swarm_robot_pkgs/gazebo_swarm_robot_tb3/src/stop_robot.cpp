/* 
 * Date: 2021-11-29
 * Description: Stop all robots
 */

//总体来说，这段代码的主要功能是创建一个ROS节点，初始化机器人控制对象，并在一个循环中不断停止一组机器人的运动。在每次循环中，它调用了swarm_robot对象的stopRobot()方法来停止所有机器人，然后暂停0.5秒。当ROS节点运行时，将一直执行这个循环，直到手动终止ROS节点。

#include <swarm_robot_control.h> // 包含自定义的头文件

/* Main function 主函数 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "stop_robot"); // 初始化ROS节点，节点名称为 "stop_robot"
    ros::NodeHandle nh; // 创建ROS节点句柄

    std::vector<int> swarm_robot_id{1, 2, 3, 4, 5}; // 创建一个包含机器人ID的向量

    SwarmRobot swarm_robot(&nh, swarm_robot_id); // 创建 SwarmRobot 类的对象，用于控制一组机器人

    /* while loop for stopping robots 用于循环停止机器人的 while 循环 */
    while(ros::ok()) { // 当ROS处于运行状态时执行以下代码

        swarm_robot.stopRobot(); // 调用 SwarmRobot 类的 stopRobot() 方法，停止所有机器人的运动
        ros::Duration(0.5).sleep(); // 暂停程序执行0.5秒，等待机器人停止

    }

    ROS_WARN_STREAM("Stop all robots!"); // 输出警告消息，表示所有机器人已停止
    return 0; // 返回0，表示程序正常结束
}
