#include<jipda/ConstantVelocityTracker.h>
#include<unsupported/Eigen/KroneckerProduct>

ConstantVelocityTracker::ConstantVelocityTracker(Eigen::VectorXd init_meas, double proc_var, double meas_var, int id){

    // TODO: parametrize matrix sizes

    // System model
    this->process_covariance_ = Eigen::MatrixXd::Identity(4,4)*proc_var;
    this->measurement_covariance_ = Eigen::MatrixXd::Identity(4,4)*meas_var;
    this->measurement_matrix_ = Eigen::MatrixXd::Identity(4,4);
    
    // Track initiation
    this->initiateTrack(init_meas);
    this->track_id_ = id;

};


visualization_msgs::Marker ConstantVelocityTracker::getStateMarker(){

    visualization_msgs::Marker return_marker;

    if(!this->estimated_state_marker_.points.empty()){

        return_marker = this->estimated_state_marker_;

    }

    else{

        geometry_msgs::Point p;
        return_marker.points.push_back(p);

    }

    // Check if empty or not after return
    return return_marker;

}


visualization_msgs::Marker ConstantVelocityTracker::getInnovationMarker(){

    visualization_msgs::Marker return_marker;

    if(!this->innovation_ellipse_marker_.points.empty()){

        return_marker = this->innovation_ellipse_marker_;

    }

    else{

        geometry_msgs::Point p;
        return_marker.points.push_back(p);

    }

    // Check if empty or not after return
    return return_marker;

}


void ConstantVelocityTracker::initiateTrack(Eigen::VectorXd init_meas){

    // TODO: Improve initialization
    
    // Naive initialization
    this->state_mean_ =  init_meas;
    int size = this->state_mean_.rows();
    this->state_covariance_ = Eigen::MatrixXd::Identity(size,size)*25;
    this->innovation_covariance_ = Eigen::MatrixXd::Identity(size/2, size/2)*5;

    // Initiate visualization
    this->initVisualization();


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
    auto state_transition_matrix_ = this->setTransitionMatrix(timestep);
    auto acceleration_matrix = this->setAccelerationMatrix(timestep);

    this->state_mean_ = state_transition_matrix_*this->state_mean_;

    this->state_covariance_ = 
        state_transition_matrix_*this->state_covariance_*state_transition_matrix_.transpose() 
        + acceleration_matrix*this->process_covariance_*acceleration_matrix.transpose();

    this->state_covariance_ = (this->state_covariance_ + this->state_covariance_.transpose())/2;  // enforce symmetry


    // Update visualization
    this->initVisualization();

};


void ConstantVelocityTracker::updateState(double timestep){

    // Prediction only
    auto state_transition_matrix_ = this->setTransitionMatrix(timestep);
    auto acceleration_matrix = this->setAccelerationMatrix(timestep);

    this->state_mean_ = state_transition_matrix_*this->state_mean_;

    this->state_covariance_ = 
        state_transition_matrix_*this->state_covariance_*state_transition_matrix_.transpose() 
        + acceleration_matrix*this->process_covariance_*acceleration_matrix.transpose();

    this->state_covariance_ = (this->state_covariance_ + this->state_covariance_.transpose())/2; // enforce symmetry

    // Update visualization
    this->initVisualization();

};


Eigen::MatrixXd ConstantVelocityTracker::setTransitionMatrix(double timestep){

    int size = this->state_mean_.rows() / 2;
    Eigen::Matrix2d single_dim_transition;
    single_dim_transition << 1, timestep, 0, 1;
    Eigen::MatrixXd identity_size_n = Eigen::MatrixXd::Identity(size, size);
    
    return Eigen::kroneckerProduct(single_dim_transition, identity_size_n); 

};


Eigen::MatrixXd ConstantVelocityTracker::setAccelerationMatrix(double timestep){

    int size = this->state_mean_.rows() / 2;
    Eigen::Vector2d single_dim_acceleration;
    single_dim_acceleration << pow(timestep, 2)/2, timestep;
    Eigen::MatrixXd identity_size_n = Eigen::MatrixXd::Identity(size, size);
    
    return Eigen::kroneckerProduct(single_dim_acceleration, identity_size_n);

};


void ConstantVelocityTracker::initVisualization(){

    this->estimated_state_marker_.header.frame_id 
        = this->innovation_ellipse_marker_.header.frame_id  = "map";

    this->estimated_state_marker_.header.stamp 
        = this->innovation_ellipse_marker_.header.stamp  = ros::Time::now();

    this->estimated_state_marker_.ns 
        = this->innovation_ellipse_marker_.ns = "jipda";
    
    this->estimated_state_marker_.action 
        = this->innovation_ellipse_marker_.action = visualization_msgs::Marker::ADD;
    
    this->estimated_state_marker_.type = visualization_msgs::Marker::CYLINDER;
    this->innovation_ellipse_marker_.type = visualization_msgs::Marker::LINE_STRIP;

    this->estimated_state_marker_.id = this->track_id_*10;
    this->innovation_ellipse_marker_.id = this->track_id_*10 + 1;
    this->estimated_state_marker_.lifetime 
        = this->innovation_ellipse_marker_.lifetime = ros::Duration();

    // facing in z direction, i.e 2D
    this->estimated_state_marker_.pose.orientation.w 
        = this->innovation_ellipse_marker_.pose.orientation.w = 1.0f;


    this->estimated_state_marker_.color.r 
        = this->innovation_ellipse_marker_.color.r = 1.0f;
    
    this->estimated_state_marker_.color.a 
        = this->innovation_ellipse_marker_.color.a = 1.0f;
    
    this->estimated_state_marker_.scale.x = 0.5f;
    this->estimated_state_marker_.scale.y = 0.5f;
    this->estimated_state_marker_.scale.z = 0.5f;
    this->innovation_ellipse_marker_.scale.x = 0.2f;
    this->innovation_ellipse_marker_.scale.y = 0.2f;
    this->innovation_ellipse_marker_.scale.z = 0.2f;

    // If exists, empty previous values
    this->estimated_state_marker_.points.clear();
    this->innovation_ellipse_marker_.points.clear();

    // Set positions as geometry_msgs::Point
    geometry_msgs::Point p;
    p.x = this->state_mean_(0);
    p.y = this->state_mean_(1);
    p.z = 0.0f;
    this->estimated_state_marker_.points.push_back(p);
    
    this->innovation_ellipse_marker_.pose.position.x = p.x;
    this->innovation_ellipse_marker_.pose.position.y = p.y;
    this->innovation_ellipse_marker_.pose.position.z = 0.0f;


    auto ellipse = this->drawEllipse(this->innovation_covariance_);
    for(int i = 0; i < 25; ++i){
        
        p.x = ellipse(0, i);
        p.y = ellipse(1, i);
        p.z = 0.0f;
        this->innovation_ellipse_marker_.points.push_back(p);

    }

    p.x = ellipse(0, 0);
    p.y = ellipse(1, 0);
    p.z = 0.0f;
    this->innovation_ellipse_marker_.points.push_back(p);

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
    std::cout << t_mat << std::endl <<std::endl;

    ellipse =  eigenvectors*eigenvalues*t_mat.transpose();
    
    std::cout << ellipse << std::endl <<std::endl;
    return ellipse;

}