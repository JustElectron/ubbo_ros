
#include "ubbo_ros/ubbo_ros.h"
#include "ubbo_ros/Proxim_sensor.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>


UbboRos::UbboRos(ros::NodeHandle& nh): _nh(nh), _priv_nh("~"){ 

    _priv_nh.param<std::string>("port", _port, "/dev/ttyACM0");
    _priv_nh.param<int>("baud", _baud, 57600);
    _priv_nh.param<std::string>("base_frame", _base_frame, "base_footprint");
    _priv_nh.param<std::string>("odom_frame", _odom_frame, "odom");
    

    _ubbo = new ubbo::Ubbo(_port, _baud);

    if (!_ubbo->isConnected()){
        if (!_ubbo->connect(_port, _baud)){
            ROS_FATAL("[UBBO] Failed to establish serial connection.\n"
            "[UBBO] Check if baud rate and port is configured correctly.");
            ros::shutdown();
        }
    }

    ROS_INFO("[UBBO] Connection established.");

    ROS_INFO_STREAM("[UBBO] Serial port " << _ubbo->getPort() << " Baud rate " << _ubbo->getBaud());

    _tf_odom.header.frame_id = _odom_frame;
    _tf_odom.child_frame_id = _base_frame;
    _odom_msg.header.frame_id = _odom_frame;
    _odom_msg.child_frame_id = _base_frame;

    _prox_sensor_msg.header.frame_id = _base_frame;

    _cmd_vel_sub = nh.subscribe("cmd_vel", 1, &UbboRos::cmdVelCallback, this);

    _odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
    _proxim_pub = nh.advertise<ubbo_ros::Proxim_sensor>("prox_sensor", 5);
}

void UbboRos::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg){
    float vel_x = msg->linear.x;
    float vel_y = msg->linear.y;
    float ang_z = msg->angular.z;

    _ubbo->drive(vel_x, vel_y, ang_z);
}

void UbboRos::publishOdom(){
    float pose_x = _ubbo->position.x;
    float pose_y = _ubbo->position.y;
    
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, _ubbo->position.yaw);
    _odom_msg.header.stamp = ros::Time::now();
    _odom_msg.pose.pose.position.x = pose_x;
    _odom_msg.pose.pose.position.y = pose_y;
    _odom_msg.pose.pose.orientation = quat;

    _tf_odom.header.stamp = ros::Time::now();
    _tf_odom.transform.translation.x = pose_x;
    _tf_odom.transform.translation.y = pose_y;
    _tf_odom.transform.rotation = quat;
    _tf_broadcaster.sendTransform(_tf_odom);

    _odom_pub.publish(_odom_msg);
}

void UbboRos::publishProximity(){
    _prox_sensor_msg.header.stamp = ros::Time::now();
    uint8_t sensor_status = _ubbo->getSensorStatus();
    _prox_sensor_msg.prox_front = sensor_status & 0x01;
    _prox_sensor_msg.prox_right = sensor_status & 0x02;
    _prox_sensor_msg.prox_back = sensor_status & 0x04;
    _prox_sensor_msg.prox_left = sensor_status & 0x08;

    _proxim_pub.publish(_prox_sensor_msg);
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "ubboRos");
    ros::NodeHandle nh;
    double loop_rate = 10.0;
    nh.param<double>("loop_rate", loop_rate, 10.0);

    UbboRos ubbo(nh);

    ros::Rate rate(loop_rate);
    
    while (ros::ok())
    {
        ubbo.publishOdom();
        ubbo.publishProximity();
        rate.sleep();

        ros::spinOnce();
    }

}
