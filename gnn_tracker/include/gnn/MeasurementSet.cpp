#include<jipda/MeasurementSet.h>

MeasurementSet::MeasurementSet(const std_msgs::Float32MultiArrayConstPtr& input, const int meas_length){

    this->input_data_ = input->data;
    this->measurement_length_ = meas_length;
    this->measurement_count_ = this->input_data_.size()/this->measurement_length_;
    this->parseInput();
    this->initVisualization();

};

Eigen::MatrixXd MeasurementSet::getParsedOutput(){

    return this->parsed_output_;

}

int MeasurementSet::getMeasurementCount(){

    return this->measurement_count_;

}

visualization_msgs::Marker MeasurementSet::getMeasPlotter(){

    return this->meas_plotter_;

}

void MeasurementSet::parseInput(){

    Eigen::VectorXd tmp_vec = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>
    (this->input_data_.data(), this->input_data_.size()).cast<double>();

    Eigen::MatrixXd tmp_matrix = Eigen::Map<Eigen::MatrixXd>
    (tmp_vec.data(), this->measurement_count_, this->measurement_length_).transpose();

    this->parsed_output_ = tmp_matrix.block(3, 0, 4, this->measurement_count_);
    this->timestamp_ = tmp_matrix.block(10, 0, 4, 1);

    // std::cout << this->parsed_output_ << std::endl << std::endl;
}

void MeasurementSet::initVisualization(){

    this->meas_plotter_.header.frame_id = "map";
    this->meas_plotter_.header.stamp = ros::Time::now();
    this->meas_plotter_.ns = "jipda";
    this->meas_plotter_.action = visualization_msgs::Marker::ADD;
    this->meas_plotter_.type = visualization_msgs::Marker::POINTS;
    this->meas_plotter_.id = 0;
    this->meas_plotter_.lifetime = ros::Duration();

    // facing in z direction, i.e 2D
    this->meas_plotter_.pose.orientation.w = 1.0f;


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
        p.z = 0.0f;
        this->meas_plotter_.points.push_back(p);

    }

}