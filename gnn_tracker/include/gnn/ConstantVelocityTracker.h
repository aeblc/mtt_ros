#pragma once
#include<Eigen/Dense>
#include<visualization_msgs/Marker.h>


// Linear Kalman filter with constant velocity model
class ConstantVelocityTracker{

public:

    ConstantVelocityTracker(Eigen::VectorXd init_meas, double proc_var, double meas_var, int id);
    void updateState(Eigen::VectorXd measurement, double timestep);
    void updateState(double timestep);


    // Interfaces
    Eigen::VectorXd getEstimatedState();
    Eigen::MatrixXd getInnovationCovariance();
    visualization_msgs::Marker getStateMarker();
    visualization_msgs::Marker getInnovationMarker();


private:

    // System dynamics model
    Eigen::VectorXd state_mean_;
    Eigen::MatrixXd state_covariance_;
    Eigen::MatrixXd process_covariance_;
    
    // Measurement model
    Eigen::MatrixXd measurement_matrix_;
    Eigen::MatrixXd measurement_covariance_;

    // Innovation covariance
    Eigen::MatrixXd innovation_covariance_;

    // Track id
    int track_id_;

    // Plotter
    visualization_msgs::Marker estimated_state_marker_;
    visualization_msgs::Marker innovation_ellipse_marker_;


    // Helper functions
    void initiateTrack(Eigen::VectorXd init_meas);
    Eigen::MatrixXd setTransitionMatrix(double timestep);
    Eigen::MatrixXd setAccelerationMatrix(double timestep);
    void initVisualization();
    Eigen::MatrixXd drawEllipse(Eigen::MatrixXd mat);
};