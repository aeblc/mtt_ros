#pragma once

#include<iostream>
#include<ros/ros.h>
#include<std_msgs/Float32MultiArray.h>
#include<visualization_msgs/Marker.h>
#include<Eigen/Dense>
#include"gnn/MeasurementSet.h"
#include"gnn/ConstantVelocityTracker.h"


class GnnNode
{
public:
    GnnNode();
    void spin();

private:

    //  Measurements 
    std::vector<MeasurementSet> all_measurements_;
    int last_processed_meas_;

    // Elapsed time between last two sets of measurements in seconds
    double timestep_;
    void updateTimestep_();

    // Trackers
    std::vector<ConstantVelocityTracker> trackers_;
    void performMeasurementAssociation_();
    Eigen::MatrixXd gnnAssociator(Eigen::MatrixXd val_mat, Eigen::MatrixXd meas);
    void updateTrackers_(Eigen::MatrixXd assoc_results, Eigen::MatrixXd meas);
    void manageTrackers_();

    // Noise models
    double process_variance_;
    double measurement_variance_;

    // Detection probability and gate threshold
    double detect_prob_;
    double gate_threshold_;

    
    // ROS node, functions, params
    ros::NodeHandle n_;
    ros::Subscriber subs_;
    std::string subs_topic_;
    ros::Publisher meas_marker_pub_;
    ros::Publisher state_marker_pub_;
    ros::Publisher cov_marker_pub_;
    int input_length_;
    int rate_;
    void subCallback_(const std_msgs::Float32MultiArrayConstPtr& scan);
    

};