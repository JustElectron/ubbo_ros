
#include "ubbo_ros/ubbo_ros.h"

UbboRos::UbboRos(ros::NodeHandle& nh): _nh(nh){ 

    nh.param<std::string>("port", _port, "/dev/ttyACM0");
    nh.param<int>("baud", _baud, 9600);

    ROS_INFO("[UBBO] Baud rate [%i]", _baud);

    _ubbo->setBaud(9600);

    _ubbo = new ubbo::Ubbo(_port, _baud);
    _ubbo->connect(_port, _baud);

    if (!_ubbo->isConnected()){
        if (!_ubbo->connect(_port, _baud)){
            ROS_FATAL("[UBBO] Failed to establish serial connection.\n"
            "[UBBO] Check if baud rate and port is configured correctly.");
            ros::shutdown();
        }
    }

    ROS_INFO("[UBBO] Connection established.");

    ROS_INFO_STREAM("[UBBO] Serial port " << _ubbo->getPort() << " Baud rate " << _ubbo->getBaud());

    _cmd_vel_sub = nh.subscribe("cmd_vel", 1, &UbboRos::cmdVelCallback, this);
}

void UbboRos::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg){
    ROS_INFO("[UBBO] cmd_vel received.");
    uint8_t forward = 90;
    uint8_t turn = 0;
    if (msg->linear.x > 0.0){
        ROS_INFO_STREAM("Subscriber velocities:"<<" linear="<<msg->linear.x<<" angular="<<msg->angular.z);
        _ubbo->driveForward(forward);
    }
    else if (msg->linear.x < 0.0){
        _ubbo->driveBackward(forward);
    }
    else if (msg->angular.z > 0.0){
        _ubbo->driveForward(turn);
    }
    else if (msg->angular.z < 0.0){
        _ubbo->driveBackward(turn);
    }
    else {

    }
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "ubboRos");
    ros::NodeHandle nh; 

    UbboRos ubbo(nh);

    ros::Rate rate(2);
    
    while (ros::ok())
    {
        rate.sleep();

        ros::spinOnce();
    }

}
