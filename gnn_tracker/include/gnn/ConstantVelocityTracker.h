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
    Eigen::RowVector2d getProbabilities();
    int getTrackID() {return this->track_id_; };
    visualization_msgs::Marker getStateMarker();
    visualization_msgs::Marker getCovMarker();
    std::tuple<Eigen::VectorXd, Eigen::MatrixXd> getPredictedMeasStats(double T);

    // Helper functions
    Eigen::MatrixXd getTransitionMatrix(double timestep);
    Eigen::MatrixXd getAccelerationMatrix(double timestep);
    Eigen::MatrixXd drawEllipse(Eigen::MatrixXd mat);
    bool isInGate(Eigen::VectorXd meas, double T, double gate_threshold);
    void deleteMarkers();
    bool isValid();
    bool isConfirmed();


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

    // Existebce probs
    Eigen::RowVector2d track_existence_probabilities_;

    // Plotter
    visualization_msgs::Marker estimated_state_marker_;
    visualization_msgs::Marker cov_ellipse_marker_;


    // Helper functions
    void initiateTrack_(Eigen::VectorXd init_meas);
    void initVisualization_();
};