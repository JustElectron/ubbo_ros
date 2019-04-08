#ifndef _UBBOROS_H_
#define _UBBOROS_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "ubbo/ubbo.h"

class UbboRos {
    private:

    ubbo::Ubbo* _ubbo;
    std::string _port;
    int _baud;

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

    protected:
    ros::NodeHandle _nh;
    ros::NodeHandle _priv_nh;
    ros::Subscriber _cmd_vel_sub;

    public:
    explicit UbboRos(ros::NodeHandle& nh);

};


#endif // _UBBOROS_H_
