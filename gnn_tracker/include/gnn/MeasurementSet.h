#pragma once
#include<ros/ros.h>
#include<std_msgs/Float32MultiArray.h>
#include<visualization_msgs/Marker.h>
#include<Eigen/Dense>

// A set of measurements
class MeasurementSet{

public:

    MeasurementSet(const std_msgs::Float32MultiArrayConstPtr& input, const int meas_length);

    // Interfaces
    int getMeasurementCount();
    Eigen::MatrixXd getParsedOutput();
    Eigen::VectorXd getTimestamp();
    visualization_msgs::Marker getMeasPlotter();




private:

    // Parsed and unparsed measurements
    int measurement_count_;
    int input_length_;
    std::vector<float> input_data_;
    Eigen::MatrixXd parsed_output_;

    // measurement timestamp in h/m/s/ms order
    Eigen::VectorXd timestamp_;

    // Rviz Plotter
    visualization_msgs::Marker meas_plotter_;

    // Helper functions

    // Parser to get positions, velocities and timestamps
    void parseInput_(); 

    // initialize visualization objects
    void initVisualization_();

};