#pragma once

#include<iostream>
#include<ros/ros.h>
#include<std_msgs/Float32MultiArray.h>
#include<visualization_msgs/Marker.h>
#include<Eigen/Dense>
#include<jipda/MeasurementSet.h>

class JipdaNode
{
public:
    JipdaNode();
    void subCallback(const std_msgs::Float32MultiArrayConstPtr& scan);
    void spin();

private:

    // 
    std::vector<MeasurementSet> all_measurements_;

    
    // ROS node, functions, params
    ros::NodeHandle n_;
    ros::Subscriber subs_;
    std::string subs_topic_;
    ros::Publisher marker_pub_;
    std::string marker_pub_topic_;
    int rate_;


};