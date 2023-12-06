/* 
 * Date: 2023-11-16
 * Description: formation control
*/

#include <swarm_robot_control.h>

/* Main function */
int main(int argc, char** argv) {
    ros::init(argc, argv, "swarm_robot_control_formation");
    ros::NodeHandle nh;
    
    /* First: Set ids of swarm robot based on Aruco marker */
    std::vector<int> swarm_robot_id{1, 2, 3, 4, 5};
    std::vector<int> dynamic_robot_id{0, 1, 2, 3, 4};   //新的队列编号

    /* Initialize swarm robot */
    SwarmRobot swarm_robot(&nh, swarm_robot_id);

    /* Set Matrix*/
    Eigen::MatrixXd lap(swarm_robot_id.size(), swarm_robot_id.size());
    lap <<  4, -1, -1, -1, -1,
            -1, 4, -1, -1, -1,
            -1, -1, 4, -1, -1,
            -1, -1, -1, 4, -1,
            -1, -1, -1, -1, 4;
    Eigen::MatrixXd adj(swarm_robot_id.size(), swarm_robot_id.size());
    adj <<  0, 1, 1, 1, 1,
            1, 0, 1, 1, 1,
            1, 1, 0, 1, 1,
            1, 1, 1, 0, 1,
            1, 1, 1, 1, 0;
    Eigen::MatrixXd d_square(swarm_robot_id.size(), swarm_robot_id.size()); //cross
    d_square << 0, 2, 1, 2, 4,
                2, 0, 1, 4, 2,
                1, 1, 0, 1, 1,
                2, 4, 1, 0, 2,
                4, 2, 1, 2, 0;  
    d_square=d_square/4;
    Eigen::MatrixXd d_square_line(swarm_robot_id.size(), swarm_robot_id.size());
    d_square_line << 0, 1, 4, 9, 16,
                    1, 0, 1, 4, 9,
                    4, 1, 0, 1, 4,
                    9, 4, 1, 0, 1,
                    16, 9, 4, 1, 0;
    d_square_line=d_square_line*pow(0.3,2);
    Eigen::MatrixXd line_x(swarm_robot_id.size(), swarm_robot_id.size());
    line_x << 0, 1, 2, 3, 4,
            -1, 0, 1, 2, 3,
            -2, -1, 0, 1, 2,
            -3, -2, -1, 0, 1,
            -4, -3, -2, -1, 0;
    line_x=line_x*0.3;
    Eigen::MatrixXd line_y(swarm_robot_id.size(), swarm_robot_id.size());
    line_y.setZero();
    Eigen::MatrixXd dline_x(swarm_robot_id.size(), swarm_robot_id.size());
    dline_x << 0, 1, 2, 0, 1,
            -1, 0, 1, -1, 0,
            -2, -1, 0, -2, -1,
            0, 1, 2, 0, 1,
            -1, 0, 1, -1, 0;
    dline_x=dline_x*0.3;
    Eigen::MatrixXd dline_y(swarm_robot_id.size(), swarm_robot_id.size());
    dline_y << 0, 0, 0, 1, 1,
            0, 0, 0, 1, 1,
            0, 0, 0, 1, 1,
            -1, -1, -1, 0, 0,
            -1, -1, -1, 0, 0;
    dline_y=dline_y*0.3;
    Eigen::MatrixXd d_square_dline(swarm_robot_id.size(), swarm_robot_id.size());
    d_square_dline << 0, 1, 4, 1, 2,
                    1, 0, 1, 2, 1,
                    4, 1, 0, 5, 2,
                    1, 2, 5, 0, 1,
                    2, 1, 2, 1, 0;
    d_square_dline=d_square_dline*pow(0.3,2);

    Eigen::MatrixXd delta_d_square(swarm_robot_id.size(), swarm_robot_id.size());
    Eigen::MatrixXd target_d(swarm_robot_id.size(), swarm_robot_id.size()); 
    Eigen::MatrixXd target_x(swarm_robot_id.size(), swarm_robot_id.size()); 
    Eigen::MatrixXd target_y(swarm_robot_id.size(), swarm_robot_id.size()); 
    /* Convergence threshold */
    double conv_th = 0.05;  // Threshold
    
    /* Velocity scale and threshold */
    double MAX_W = 1;       // Maximum angle velocity (rad/s)
    double MIN_W = 0.05;    // Minimum angle velocity(rad/s)
    double MAX_V = 0.2;     // Maximum linear velocity(m/s)
    double MIN_V = 0.01;    // Minimum linear velocity(m/s)
    double k_w = 0.1;       // Scale of angle velocity
    double k_v = 0.1;       // Scale of linear velocity
    double k_d = 1;       // Scale of distance
    double pi = std::acos(-1.0);
    double w, v;

    d_square *= std::pow(k_d,2);

    /* Mobile robot poses and for next poses */
    Eigen::VectorXd cur_x(swarm_robot_id.size());
    Eigen::VectorXd cur_y(swarm_robot_id.size());
    Eigen::VectorXd cur_theta(swarm_robot_id.size());
    Eigen::VectorXd del_x(swarm_robot_id.size());
    Eigen::VectorXd del_y(swarm_robot_id.size());
    Eigen::VectorXd del_theta(swarm_robot_id.size());
    Eigen::VectorXd v_theta(swarm_robot_id.size());

    /* Get swarm robot poses firstly */
    std::vector<std::vector<double> > current_robot_pose(swarm_robot_id.size());

    swarm_robot.getRobotPose(current_robot_pose);

    for(int i = 0; i < swarm_robot_id.size(); i++) {
        cur_x(i) = current_robot_pose[i][0];
        cur_y(i) = current_robot_pose[i][1];
    }
    
    // cross or T
    std::sort(dynamic_robot_id.begin(), dynamic_robot_id.end(), [&cur_x](int i1, int i2) {return cur_x(i1) < cur_x(i2);});  
    std::sort(dynamic_robot_id.begin()+1, dynamic_robot_id.end()-1, [&cur_y](int i1, int i2) {return cur_y(i1) < cur_y(i2);});  
    // wedge
    //std::sort(dynamic_robot_id.begin(), dynamic_robot_id.end(), [&cur_x](int i1, int i2) {return cur_x(i1) < cur_x(i2);});

    for(int i = 0; i < swarm_robot_id.size(); i++) {
        cur_theta(i) = current_robot_pose[dynamic_robot_id[i]][2];
        cur_x(i) = current_robot_pose[dynamic_robot_id[i]][0];
        cur_y(i) = current_robot_pose[dynamic_robot_id[i]][1];
    }
    /*--------------------------------------------------------------------------move to target formation-----------------------------------------------------------------------------------*/
    /* Convergence sign */
    bool is_conv = false;   // Convergence sign of angle
    /* While loop */
    while(! is_conv) {
        /* Judge whether reached */
        for (int i = 0; i < swarm_robot_id.size(); i++) {
            for (int j = 0; j < swarm_robot_id.size(); j++) {
                delta_d_square(i,j) = adj(i,j) * (std::pow(cur_x(i)-cur_x(j),2) + std::pow(cur_y(i)-cur_y(j),2) - d_square(i,j));
            }
        }

        for (int i = 0; i < swarm_robot_id.size(); i++) {
            del_x(i) = 0;
            del_y(i) = 0;
            for (int j = 0; j < swarm_robot_id.size(); j++) {
                del_x(i) = del_x(i) - delta_d_square(i,j) * (cur_x(i) - cur_x(j));
                del_y(i) = del_y(i) - delta_d_square(i,j) * (cur_y(i) - cur_y(j));
            }
            v_theta(i) = std::atan2(del_y(i) , del_x(i));
            del_theta(i) = -(cur_theta(i) - v_theta(i));
            while (del_theta(i) < -pi or del_theta(i) > pi) {
                if (del_theta(i) < -pi) del_theta(i) += 2 * pi;
                if (del_theta(i) > pi) del_theta(i) -= 2 * pi;
            }           
        }

        is_conv = true;
        if (delta_d_square.norm() > conv_th*2) is_conv = false;

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
            swarm_robot.moveRobot(dynamic_robot_id[i], v, w);  
        }

        /* Time sleep for robot move */
        ros::Duration(0.05).sleep();

        /* Get swarm robot poses */
        swarm_robot.getRobotPose(current_robot_pose);
        
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            cur_theta(i) = current_robot_pose[dynamic_robot_id[i]][2];
            cur_x(i) = current_robot_pose[dynamic_robot_id[i]][0];
            cur_y(i) = current_robot_pose[dynamic_robot_id[i]][1];
        }
    }

    bool is_conv_theta = false;             //angle

    while(! is_conv_theta) {
        is_conv_theta = true;
        
        del_theta = -lap * cur_theta;
        del_theta(0)=0-cur_theta(0);
        double w0 = del_theta(0)*k_w*2;
        w0 = swarm_robot.checkVel(w0, MAX_W, MIN_W);
        swarm_robot.moveRobot(dynamic_robot_id[0], 0.0, w0);
        
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            if(std::fabs(del_theta(i)) > conv_th*1.5) {
                is_conv_theta = false;
            }       
        }
        
        for(int i = 1; i < swarm_robot_id.size(); i++) {
            double w = del_theta(i) * k_w;
            w = swarm_robot.checkVel(w, MAX_W, MIN_W);
            swarm_robot.moveRobot(dynamic_robot_id[i], 0.0, w);
        }

        ros::Duration(0.05).sleep();
        swarm_robot.getRobotPose(current_robot_pose);
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            cur_theta(i) = current_robot_pose[dynamic_robot_id[i]][2];
        }
    }

    double length=cur_x(0)+0.5;
    bool is_conv_length = false;
    while(! is_conv_length){
        is_conv_length = true;
        if(std::fabs(cur_x(0)-length) > conv_th){
            is_conv_length = false;
        }
        for(int i=0; i<swarm_robot_id.size(); i++){
            swarm_robot.moveRobot(dynamic_robot_id[i], 0.1, 0);
        }
        ros::Duration(0.05).sleep();
        swarm_robot.getRobotPose(current_robot_pose);
        cur_x(0)=current_robot_pose[dynamic_robot_id[0]][0];
    }
 
    /*------------------------------------------------------------------------------choose formation----------------------------------------------------------------------------------------*/
    //swarm_robot.stopRobot();
    ros::Duration(2).sleep();
    swarm_robot.getRobotPose(current_robot_pose);
    for(int i = 0; i < swarm_robot_id.size(); i++) {
        cur_theta(i) = current_robot_pose[i][2];
        cur_x(i) = current_robot_pose[i][0];
        cur_y(i) = current_robot_pose[i][1];
    }
    Eigen::VectorXd epsilon_line(swarm_robot_id.size());
    Eigen::VectorXd epsilon_dline(swarm_robot_id.size());
    Eigen::VectorXd zeta_line(swarm_robot_id.size());
    Eigen::VectorXd zeta_dline(swarm_robot_id.size());
    Eigen::VectorXd del_zeta_line(swarm_robot_id.size());
    Eigen::VectorXd del_zeta_dline(swarm_robot_id.size());
    epsilon_line.setZero();
    epsilon_dline.setZero();
    for(int i=0; i<swarm_robot_id.size(); i++){         //set zeta(0)
        for(int j=0; j<swarm_robot_id.size(); j++){
            epsilon_line(i)=epsilon_line(i)+adj(i,j)*pow(pow(cur_x(i)-cur_x(j),2)+pow(cur_y(i)-cur_y(j),2)-d_square_line(i,j),2);
            epsilon_dline(i)=epsilon_dline(i)+adj(i,j)*pow(pow(cur_x(i)-cur_x(j),2)+pow(cur_y(i)-cur_y(j),2)-d_square_dline(i,j),2);
        }
        zeta_line(i)=epsilon_line(i);
        zeta_dline(i)=epsilon_dline(i);
    }

    for(int k=0; k<100; k++){
        del_zeta_line.setZero();
        del_zeta_dline.setZero();
        for(int i=0; i<swarm_robot_id.size(); i++){
            for(int j=0; j<swarm_robot_id.size(); j++){
                del_zeta_line(i)=del_zeta_line(i)-(zeta_line(i)-zeta_line(j));
                del_zeta_dline(i)=del_zeta_dline(i)-(zeta_dline(i)-zeta_dline(j));
            }
        }
        zeta_line=zeta_line+del_zeta_line;
        zeta_dline=zeta_dline+del_zeta_dline;
        cout<<zeta_line<<endl;
        cout<<zeta_dline<<endl;
    }
    if(zeta_line.norm() < zeta_dline.norm()){     //choose formation
        target_d=d_square_line;
        target_x=line_x;
        target_y=line_y;
        std::sort(dynamic_robot_id.begin(), dynamic_robot_id.end(), [&cur_x](int i1, int i2) {return cur_x(i1) > cur_x(i2);}); 
    }
    else{
        target_d=d_square_dline;
        target_x=dline_x;
        target_y=dline_y;
        std::sort(dynamic_robot_id.begin(), dynamic_robot_id.end(), [&cur_y](int i1, int i2) {return cur_y(i1) > cur_y(i2);});
        std::sort(dynamic_robot_id.begin(), dynamic_robot_id.begin()+2, [&cur_x](int i1, int i2) {return cur_x(i1) > cur_x(i2);});
        std::sort(dynamic_robot_id.begin()+3, dynamic_robot_id.end(), [&cur_x](int i1, int i2) {return cur_x(i1) > cur_x(i2);});
    }

    /*------------------------------------------------------------------------move to line/double_line---------------------------------------------------------------------------------------------*/

    for(int i = 0; i < swarm_robot_id.size(); i++) {
        cur_theta(i) = current_robot_pose[dynamic_robot_id[i]][2];
        cur_x(i) = current_robot_pose[dynamic_robot_id[i]][0];
        cur_y(i) = current_robot_pose[dynamic_robot_id[i]][1];
    }

    bool is_conv2 = false;
    while(! is_conv2) {
        /* Judge whether reached */
        for (int i = 0; i < swarm_robot_id.size(); i++) {
            for (int j = 0; j < swarm_robot_id.size(); j++) {
                delta_d_square(i,j) = adj(i,j) * (std::pow(cur_x(i)-cur_x(j),2) + std::pow(cur_y(i)-cur_y(j),2) - target_d(i,j));
            }
        }

        for (int i = 0; i < swarm_robot_id.size(); i++) {
            del_x(i) = 0;
            del_y(i) = 0;
            for (int j = 0; j < swarm_robot_id.size(); j++) {
                del_x(i) = del_x(i) - delta_d_square(i,j) * (cur_x(i) - cur_x(j))-(cur_x(i)-cur_x(j)-target_x(i,j));
                del_y(i) = del_y(i) - delta_d_square(i,j) * (cur_y(i) - cur_y(j))-(cur_y(i)-cur_y(j)-target_y(i,j));
            }
            v_theta(i) = std::atan2(del_y(i) , del_x(i));
            del_theta(i) = -(cur_theta(i) - v_theta(i));
            while (del_theta(i) < -pi or del_theta(i) > pi) {
                if (del_theta(i) < -pi) del_theta(i) += 2 * pi;
                if (del_theta(i) > pi) del_theta(i) -= 2 * pi;
            }           
        }

        is_conv2 = true;
        if (delta_d_square.norm() > conv_th) is_conv2 = false;

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
            swarm_robot.moveRobot(dynamic_robot_id[i], v, w);   
        }

        /* Time sleep for robot move */
        ros::Duration(0.05).sleep();

        /* Get swarm robot poses */
        swarm_robot.getRobotPose(current_robot_pose);
        
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            cur_theta(i) = current_robot_pose[dynamic_robot_id[i]][2];
            cur_x(i) = current_robot_pose[dynamic_robot_id[i]][0];
            cur_y(i) = current_robot_pose[dynamic_robot_id[i]][1];
        }
    }

    is_conv_theta = false;          //angle
    while(! is_conv_theta) {
        is_conv_theta = true;
        
        del_theta = -lap * cur_theta;
        del_theta(0)=0-cur_theta(0);
        double w0 = del_theta(0)*k_w*2;
        w0 = swarm_robot.checkVel(w0, MAX_W, MIN_W);
        swarm_robot.moveRobot(dynamic_robot_id[0], 0.0, w0);

        for(int i = 0; i < swarm_robot_id.size(); i++) {
            if(std::fabs(del_theta(i)) > conv_th*1.5) {
                is_conv_theta = false;
            }       
        }
        /* Swarm robot move */
        for(int i = 1; i < swarm_robot_id.size(); i++) {
            double w = del_theta(i) * k_w;
            w = swarm_robot.checkVel(w, MAX_W, MIN_W);
            swarm_robot.moveRobot(dynamic_robot_id[i], 0.0, w);
        }

        ros::Duration(0.05).sleep();
        swarm_robot.getRobotPose(current_robot_pose);
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            cur_theta(i) = current_robot_pose[dynamic_robot_id[i]][2];
        }
    }

    length=cur_x(0)+1.4;
    is_conv_length = false;
    while(! is_conv_length){
        is_conv_length = true;
        if(std::fabs(cur_x(0)-length) > conv_th){
            is_conv_length = false;
        }
        for(int i=0; i<swarm_robot_id.size(); i++){
            swarm_robot.moveRobot(dynamic_robot_id[i], 0.1, 0);
        }
        ros::Duration(0.05).sleep();
        swarm_robot.getRobotPose(current_robot_pose);
        cur_x(0)=current_robot_pose[dynamic_robot_id[0]][0];
    }
/*-----------------------------------------------------------------------------------move back to target formation-----------------------------------------------------------------------------------------------------*/
    for(int i = 0; i < swarm_robot_id.size(); i++) {
        cur_theta(i) = current_robot_pose[dynamic_robot_id[i]][2];
        cur_x(i) = current_robot_pose[dynamic_robot_id[i]][0];
        cur_y(i) = current_robot_pose[dynamic_robot_id[i]][1];
    }

    is_conv = false;   // Convergence sign of angle
    /* While loop */
    while(! is_conv) {
        /* Judge whether reached */
        for (int i = 0; i < swarm_robot_id.size(); i++) {
            for (int j = 0; j < swarm_robot_id.size(); j++) {
                delta_d_square(i,j) = adj(i,j) * (std::pow(cur_x(i)-cur_x(j),2) + std::pow(cur_y(i)-cur_y(j),2) - d_square(i,j));
            }
        }

        for (int i = 0; i < swarm_robot_id.size(); i++) {
            del_x(i) = 0;
            del_y(i) = 0;
            for (int j = 0; j < swarm_robot_id.size(); j++) {
                del_x(i) = del_x(i) - delta_d_square(i,j) * (cur_x(i) - cur_x(j));
                del_y(i) = del_y(i) - delta_d_square(i,j) * (cur_y(i) - cur_y(j));
            }
            v_theta(i) = std::atan2(del_y(i) , del_x(i));
            del_theta(i) = -(cur_theta(i) - v_theta(i));
            while (del_theta(i) < -pi or del_theta(i) > pi) {
                if (del_theta(i) < -pi) del_theta(i) += 2 * pi;
                if (del_theta(i) > pi) del_theta(i) -= 2 * pi;
            }           
        }

        is_conv = true;
        if (delta_d_square.norm() > conv_th*2) is_conv = false;

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
            swarm_robot.moveRobot(dynamic_robot_id[i], v, w);  
        }

        /* Time sleep for robot move */
        ros::Duration(0.05).sleep();

        /* Get swarm robot poses */
        swarm_robot.getRobotPose(current_robot_pose);
        
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            cur_theta(i) = current_robot_pose[dynamic_robot_id[i]][2];
            cur_x(i) = current_robot_pose[dynamic_robot_id[i]][0];
            cur_y(i) = current_robot_pose[dynamic_robot_id[i]][1];
        }
    }

    
    /* Stop all robots */
    swarm_robot.stopRobot();
    if(zeta_line.norm() < zeta_dline.norm()){     
        ROS_INFO_STREAM("formation_line!");
    }
    else{
        ROS_INFO_STREAM("formation_double_line!");
    }
    ROS_INFO_STREAM("Succeed!");
    return 0;
}

