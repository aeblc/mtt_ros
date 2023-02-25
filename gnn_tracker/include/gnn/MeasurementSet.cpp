#include"gnn/MeasurementSet.h"

MeasurementSet::MeasurementSet(const std_msgs::Float32MultiArrayConstPtr& input, const int input_length){

    this->input_data_ = input->data;
    this->input_length_ = input_length;
    this->measurement_count_ = this->input_data_.size()/this->input_length_;
    this->parseInput_();
    this->initVisualization_();

};

// Interfaces
Eigen::MatrixXd MeasurementSet::getParsedOutput(){

    return this->parsed_output_;

}

Eigen::VectorXd MeasurementSet::getTimestamp(){

    return this->timestamp_;

}

int MeasurementSet::getMeasurementCount(){

    return this->measurement_count_;

}

visualization_msgs::Marker MeasurementSet::getMeasPlotter(){

    return this->meas_plotter_;

}

// Parses the input to position and velocity measurements
void MeasurementSet::parseInput_(){

    // Map std::vector to eigen::vector, cast to double
    Eigen::VectorXd tmp_vec = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>
    (this->input_data_.data(), this->input_data_.size()).cast<double>();

    // Rearrange into a matrix for convenient block operations
    Eigen::MatrixXd tmp_matrix = Eigen::Map<Eigen::MatrixXd>
    (tmp_vec.data(), this->measurement_count_, this->input_length_).transpose();

    // Remove possible duplicate measurements
    Eigen::MatrixXd tentative_meas = tmp_matrix.block(3, 0, 4, this->measurement_count_);
    std::vector<int> valid_indices;
    
    // always begin picking the first measurement
    valid_indices.push_back(0);
    if(this->measurement_count_ > 1)
    for(int i=1; i<tentative_meas.rows()-1; ++i){
        bool flag = true;
        for(int j=i+1; j<tentative_meas.cols(); ++j){

            if((tentative_meas.col(i) - tentative_meas.col(j)).norm() < 1e-3){

                flag = false;
                break;

            }
        }

        if(flag)
            valid_indices.push_back(i);
    }


    // Get position-velocity measurements
    this->parsed_output_.resize(4, valid_indices.size());
    for (auto idx: valid_indices)
        this->parsed_output_ << tentative_meas.col(idx);

    this->measurement_count_ = this->parsed_output_.cols();

    // Obtain timestamps
    this->timestamp_ = tmp_matrix.block(10, 0, 4, 1);

}


// Constructs marker messages for plotting in Rviz
void MeasurementSet::initVisualization_(){

    // If exists, empty previous values
    this->meas_plotter_.points.clear();

    // Begin constructing messages
    this->meas_plotter_.header.frame_id = "map";
    this->meas_plotter_.header.stamp = ros::Time::now();
    this->meas_plotter_.ns = "gnn";
    this->meas_plotter_.action = visualization_msgs::Marker::ADD;
    this->meas_plotter_.type = visualization_msgs::Marker::POINTS;
    this->meas_plotter_.id = 0;
    this->meas_plotter_.lifetime = ros::Duration();

    // facing in z direction, i.e 2D
    this->meas_plotter_.pose.orientation.w = 1.0f;

    // Set colors and sizes
    this->meas_plotter_.color.g = 1.0f;
    this->meas_plotter_.color.a = 1;
    this->meas_plotter_.scale.x = 0.2f;
    this->meas_plotter_.scale.y = 0.2f;

    // Set positions as geometry_msgs::Point
    for(int i = 0; i < this->measurement_count_; ++i){

        float x = this->parsed_output_(0, i);
        float y = this->parsed_output_(1, i);

        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        p.z = 0.5f;
        this->meas_plotter_.points.push_back(p);

    }

}