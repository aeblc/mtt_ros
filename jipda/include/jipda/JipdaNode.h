#pragma once

#include<iostream>
#include<ros/ros.h>
#include<std_msgs/Float32MultiArray.h>
#include<visualization_msgs/Marker.h>
#include<Eigen/Dense>
#include<jipda/MeasurementSet.h>
#include<jipda/ConstantVelocityTracker.h>


class JipdaNode
{
public:
    JipdaNode();
    void spin();

private:

    //  Measurements 
    int measurement_length_;
    std::vector<MeasurementSet> all_measurements_;

    // Trackers
    std::vector<ConstantVelocityTracker> trackers_;

    // Noise models
    double process_variance_;
    double measurement_variance_;

    
    // ROS node, functions, params
    ros::NodeHandle n_;
    ros::Subscriber subs_;
    std::string subs_topic_;
    ros::Publisher meas_marker_pub_;
    ros::Publisher state_marker_pub_;
    ros::Publisher innovation_marker_pub_;
    int rate_;
    void subCallback(const std_msgs::Float32MultiArrayConstPtr& scan);

};