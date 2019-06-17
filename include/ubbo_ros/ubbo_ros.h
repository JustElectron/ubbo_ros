#ifndef _UBBOROS_H_
#define _UBBOROS_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include "ubbo/ubbo.h"
#include "ubbo_ros/Proxim_sensor.h"

class UbboRos {
    private:

    ubbo::Ubbo* _ubbo;
    std::string _port;
    int _baud;
    std::string _base_frame;
    std::string _odom_frame;

    ubbo_ros::Proxim_sensor _prox_sensor_msg;

    nav_msgs::Odometry _odom_msg;
    tf::TransformBroadcaster _tf_broadcaster;
    geometry_msgs::TransformStamped _tf_odom;

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

    protected:
    ros::NodeHandle _nh;
    ros::NodeHandle _priv_nh;

    ros::Subscriber _cmd_vel_sub;
    
    ros::Publisher _odom_pub;
    ros::Publisher _proxim_pub;

    public:
    explicit UbboRos(ros::NodeHandle& nh);
    void publishOdom();
    void publishProximity();

};


#endif // _UBBOROS_H_
