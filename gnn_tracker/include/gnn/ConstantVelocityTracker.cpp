#include"gnn/ConstantVelocityTracker.h"
#include<unsupported/Eigen/KroneckerProduct>


ConstantVelocityTracker::ConstantVelocityTracker(Eigen::VectorXd init_meas, double proc_var, double meas_var, int id){

    // TODO: parametrize matrix sizes

    // System model
    this->process_covariance_ = Eigen::MatrixXd::Identity(4,4)*proc_var;
    this->measurement_covariance_ = Eigen::MatrixXd::Identity(4,4)*meas_var;
    this->measurement_matrix_ = Eigen::MatrixXd::Identity(4,4);
    
    // Track initiation
    this->initiateTrack_(init_meas);
    this->track_id_ = id;

};

Eigen::RowVector2d ConstantVelocityTracker::getProbabilities(){

    return this->track_existence_probabilities_;

}

visualization_msgs::Marker ConstantVelocityTracker::getStateMarker(){
    return this->estimated_state_marker_;
}


visualization_msgs::Marker ConstantVelocityTracker::getCovMarker(){
    return this->cov_ellipse_marker_;
}

bool ConstantVelocityTracker::isConfirmed(){

    if(this->track_existence_probabilities_(0) > 0.5)
        return true;
    else
        return false;

}

bool ConstantVelocityTracker::isValid(){

    if(this->track_existence_probabilities_(0) > 0.1)
        return true;
    else
        return false;

}

bool ConstantVelocityTracker::isInGate(Eigen::VectorXd meas, double T, double gate_threshold){

    Eigen::VectorXd pred_meas;
    Eigen::MatrixXd innov_cov;
    std::tie(pred_meas, innov_cov) = this->getPredictedMeasStats(T);

    // Calculate quadratic distance
    double quad_dist 
        = (meas - pred_meas).transpose() * innov_cov.inverse() * (meas - pred_meas);
    
    if(quad_dist < gate_threshold)
        return true;
    else
        return false;

}


void ConstantVelocityTracker::initiateTrack_(Eigen::VectorXd init_meas){

    // TODO: Improve initialization
    
    // Naïve initialization
    this->state_mean_ =  init_meas;
    int size = this->state_mean_.rows();
    this->state_covariance_ = Eigen::MatrixXd::Identity(size,size)*25;
    this->innovation_covariance_ = Eigen::MatrixXd::Identity(size/2, size/2);
    double init_existance_prob = .25;
    this->track_existence_probabilities_ << init_existance_prob, 1-init_existance_prob;

    // Initiate visualization
    this->initVisualization_();

};


void ConstantVelocityTracker::updateState(Eigen::VectorXd measurement, double timestep){

    // Correction
    this->innovation_covariance_ = 
        this->measurement_matrix_*this->state_covariance_*this->measurement_matrix_.transpose() 
        + this->measurement_covariance_;

    auto innovation_inv = this->innovation_covariance_.inverse();

    this->state_mean_ = 
        this->state_mean_ + this->state_covariance_*this->measurement_matrix_.transpose()
        *innovation_inv*(measurement - this->measurement_matrix_*this->state_mean_);

    this->state_covariance_ = 
        this->state_covariance_ - this->state_covariance_*this->measurement_matrix_.transpose()
        *innovation_inv*this->measurement_matrix_*this->state_covariance_;
    
    this->state_covariance_ = (this->state_covariance_ + this->state_covariance_.transpose())/2;  // enforce symmetry


    // Prediction
    auto state_transition_matrix_ = this->getTransitionMatrix(timestep);
    auto acceleration_matrix = this->getAccelerationMatrix(timestep);

    this->state_mean_ = state_transition_matrix_*this->state_mean_;

    this->state_covariance_ = 
        state_transition_matrix_*this->state_covariance_*state_transition_matrix_.transpose() 
        + acceleration_matrix*this->process_covariance_*acceleration_matrix.transpose();

    this->state_covariance_ = (this->state_covariance_ + this->state_covariance_.transpose())/2;  // enforce symmetry


    // Update prob
    // TODO: Improve prob update
    double updated_prob = this->track_existence_probabilities_[0]+.1;
    this->track_existence_probabilities_[0] = std::min(1.0, updated_prob) ;
    this->track_existence_probabilities_[1] = 1 - this->track_existence_probabilities_[0];

    // Update visualization
    this->initVisualization_();

};


void ConstantVelocityTracker::updateState(double timestep){

    // Prediction only
    auto state_transition_matrix_ = this->getTransitionMatrix(timestep);
    auto acceleration_matrix = this->getAccelerationMatrix(timestep);

    this->state_mean_ = state_transition_matrix_*this->state_mean_;

    this->state_covariance_ = 
        state_transition_matrix_*this->state_covariance_*state_transition_matrix_.transpose() 
        + acceleration_matrix*this->process_covariance_*acceleration_matrix.transpose();

    this->state_covariance_ = (this->state_covariance_ + this->state_covariance_.transpose())/2; // enforce symmetry

    // Update prob
    // TODO: Improve prob update
    double updated_prob = this->track_existence_probabilities_[0]-.1;
    this->track_existence_probabilities_[0] = std::max(0.0, updated_prob) ;
    this->track_existence_probabilities_[1] = 1 - this->track_existence_probabilities_[0];

    // Update visualization
    this->initVisualization_();

};


Eigen::MatrixXd ConstantVelocityTracker::getTransitionMatrix(double timestep){

    int size = this->state_mean_.rows() / 2;
    Eigen::Matrix2d single_dim_transition;
    single_dim_transition << 1, timestep, 0, 1;
    Eigen::MatrixXd identity_size_n = Eigen::MatrixXd::Identity(size, size);
    
    return Eigen::kroneckerProduct(single_dim_transition, identity_size_n); 

};


Eigen::MatrixXd ConstantVelocityTracker::getAccelerationMatrix(double timestep){

    int size = this->state_mean_.rows() / 2;
    Eigen::Vector2d single_dim_acceleration;
    single_dim_acceleration << pow(timestep, 2)/2, timestep;
    Eigen::MatrixXd identity_size_n = Eigen::MatrixXd::Identity(size, size);
    
    return Eigen::kroneckerProduct(single_dim_acceleration, identity_size_n);

};


std::tuple<Eigen::VectorXd, Eigen::MatrixXd> ConstantVelocityTracker::
    getPredictedMeasStats(double T){
    
    // get transition and acceleration matrices
    auto tran_mat = this->getTransitionMatrix(T);
    auto accel_mat = this->getAccelerationMatrix(T);

    // compute predicted mean and covariance
    auto pred_mean = tran_mat*this->state_mean_;
    auto pred_cov = 
        tran_mat*this->state_covariance_*tran_mat.transpose() 
        + accel_mat*this->process_covariance_*accel_mat.transpose();

    // Compute predicted mean and cov for the measurement
    auto pred_meas = this->measurement_matrix_*pred_mean;
    auto innov_cov = 
        this->measurement_matrix_*pred_cov*this->measurement_matrix_.transpose() 
        + this->measurement_covariance_;
    
    return std::make_tuple(pred_meas, innov_cov);
};

void ConstantVelocityTracker::initVisualization_(){

    // If exists, empty previous values
    this->estimated_state_marker_.points.clear();
    this->cov_ellipse_marker_.points.clear();


    this->estimated_state_marker_.header.frame_id 
        = this->cov_ellipse_marker_.header.frame_id  = "map";

    this->estimated_state_marker_.header.stamp 
        = this->cov_ellipse_marker_.header.stamp  = ros::Time::now();

    this->estimated_state_marker_.ns 
        = this->cov_ellipse_marker_.ns = "gnn";
    
    this->estimated_state_marker_.action 
        = this->cov_ellipse_marker_.action = visualization_msgs::Marker::ADD;
    
    this->estimated_state_marker_.type = visualization_msgs::Marker::SPHERE;
    this->cov_ellipse_marker_.type = visualization_msgs::Marker::LINE_STRIP;

    this->estimated_state_marker_.id = this->track_id_;
    this->cov_ellipse_marker_.id = this->track_id_;
    this->estimated_state_marker_.lifetime 
        = this->cov_ellipse_marker_.lifetime = ros::Duration();

    // facing in z direction, i.e 2D
    this->estimated_state_marker_.pose.orientation.w 
        = this->cov_ellipse_marker_.pose.orientation.w = 1.0f;


    this->estimated_state_marker_.color.r 
        = this->cov_ellipse_marker_.color.r = 1.0f;
    
    this->estimated_state_marker_.color.a 
        = this->cov_ellipse_marker_.color.a = 1.0f;
    
    this->estimated_state_marker_.scale.x = 0.5f;
    this->estimated_state_marker_.scale.y = 0.5f;
    this->estimated_state_marker_.scale.z = 0.5f;
    this->cov_ellipse_marker_.scale.x = 0.2f;
    this->cov_ellipse_marker_.scale.y = 0.2f;
    this->cov_ellipse_marker_.scale.z = 0.2f;

    // Set positions
    this->estimated_state_marker_.pose.position.x = this->state_mean_(0);
    this->estimated_state_marker_.pose.position.y = this->state_mean_(1);
    this->estimated_state_marker_.pose.position.z = 0.1f;
    
    this->cov_ellipse_marker_.pose.position.x = this->state_mean_(0);
    this->cov_ellipse_marker_.pose.position.y = this->state_mean_(1);
    this->cov_ellipse_marker_.pose.position.z = 0.1f;

    geometry_msgs::Point p;
    auto ellipse = this->drawEllipse(this->innovation_covariance_);
    for(int i = 0; i < 25; ++i){
        
        p.x = ellipse(0, i);
        p.y = ellipse(1, i);
        p.z = 0.0f;
        this->cov_ellipse_marker_.points.push_back(p);

    }

    p.x = ellipse(0, 0);
    p.y = ellipse(1, 0);
    p.z = 0.0f;
    this->cov_ellipse_marker_.points.push_back(p);

}


Eigen::MatrixXd ConstantVelocityTracker::drawEllipse(Eigen::MatrixXd mat){

    // Get eigendecomposition
    auto position_mat = mat.block(0,0,2,2);
    Eigen::EigenSolver<Eigen::MatrixXd> solver(position_mat);
    solver.compute(position_mat, true);
    Eigen::MatrixXd eigenvalues = solver.eigenvalues().real().asDiagonal();
    Eigen::MatrixXd eigenvectors = solver.eigenvectors().real();
    eigenvalues = eigenvalues.array().sqrt().matrix().real() ;

    // Pick points from 0 to 2*pi
    int n = 25;
    Eigen::VectorXd t = Eigen::VectorXd::LinSpaced(n,0,2*M_PI);
    Eigen::MatrixXd ellipse(2, n);

    Eigen::MatrixXd t_mat(n, 2);
    t_mat << t.array().cos().matrix(), t.array().sin().matrix();

    ellipse =  eigenvectors*eigenvalues*t_mat.transpose();
    
    return ellipse;

}

void ConstantVelocityTracker::deleteMarkers(){

    this->estimated_state_marker_.action = this->cov_ellipse_marker_.action =
        visualization_msgs::Marker::DELETE;

}