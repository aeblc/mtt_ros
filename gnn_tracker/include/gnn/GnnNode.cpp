#include"GnnNode.h"
#include"munkres-cpp/adapters/matrix_eigen.h"
#include"munkres-cpp/munkres.h"

GnnNode::GnnNode() {

   // Gather parameters from the launch file
   this->n_.getParam("/gnn_node/subs_topic", this->subs_topic_);
   this->n_.getParam("/gnn_node/rate", this->rate_);
   this->n_.getParam("/gnn_node/input_length", this->input_length_);
   this->n_.getParam("/gnn_node/proc_var", this->process_variance_);
   this->n_.getParam("/gnn_node/meas_var", this->measurement_variance_);
   this->n_.getParam("/gnn_node/detect_prob", this->detect_prob_);
   this->n_.getParam("/gnn_node/gate_threshold", this->gate_threshold_);

   ROS_INFO("My node is alive!!");

   // Initialize subscribers and publishers
   this->subs_ = this->n_.subscribe(this->subs_topic_, 10, &GnnNode::subCallback_, this);
   this->meas_marker_pub_ = this->n_.advertise<visualization_msgs::Marker>("/gnn/measurements", 10, this);
   this->state_marker_pub_ = this->n_.advertise<visualization_msgs::Marker>("/gnn/estimated_state", 10, this);
   this->cov_marker_pub_ = this->n_.advertise<visualization_msgs::Marker>("/gnn/innovation_cov", 10, this);

   this->last_processed_meas_ = 0;

};


// Main loop of the GNN class
void GnnNode::spin()
{
   ros::Rate rate(this->rate_);

   while (ros::ok()){
      ros::spinOnce();


      if (!this->all_measurements_.empty()){
         rate.sleep();
         this->meas_marker_pub_.publish(this->all_measurements_[this->last_processed_meas_].getMeasPlotter());

         if (this->all_measurements_.size() > this->last_processed_meas_ )
            this->performMeasurementAssociation_();
      }

      if (!this->trackers_.empty()){

         for (int i = 0; i < this->trackers_.size(); ++i){
            if(!this->trackers_[i].isValid()){
               this->trackers_[i].deleteMarkers();
               this->state_marker_pub_.publish(this->trackers_[i].getStateMarker());
               this->cov_marker_pub_.publish(this->trackers_[i].getCovMarker());
            }

            if(this->trackers_[i].isConfirmed()){
            this->state_marker_pub_.publish(this->trackers_[i].getStateMarker());
            this->cov_marker_pub_.publish(this->trackers_[i].getCovMarker());
            }
         }

      this->manageTrackers_();

      }

      rate.sleep();
      
   }
}


void GnnNode::performMeasurementAssociation_(){

   // Use the latest measurements
   auto meas = this->all_measurements_[this->last_processed_meas_].getParsedOutput();
   this->updateTimestep_();
   this->last_processed_meas_ += 1; 

   // If no tracks have been initiated yet, initiate tracks with all measurements
   if (this->trackers_.empty()){
      for(int i=0; i<meas.cols(); ++i){
         auto tr = ConstantVelocityTracker(meas.col(i), this->process_variance_, this->measurement_variance_, i);
         this->trackers_.push_back(tr);
      }
      return;
   }


   // Check if measurements are in the gate
   Eigen::MatrixXd validation_matrix(meas.cols(), this->trackers_.size());
   for(int i = 0; i < meas.cols(); ++i){
      for(int j = 0; j < this->trackers_.size(); ++j){
         if(this->trackers_[j].isInGate(meas.col(i), this->timestep_, this->gate_threshold_))
            validation_matrix(i,j) = 1;
         else 
            validation_matrix(i,j) = 0;
      }
   }

   auto assoc_results = this->gnnAssociator(validation_matrix, meas);

   // Perform track updates using the associations
   this->updateTrackers_(assoc_results, meas);

}


Eigen::MatrixXd GnnNode::gnnAssociator(Eigen::MatrixXd val_mat, Eigen::MatrixXd meas){


   // fill up the association matrix
   Eigen::VectorXd mean;
   Eigen::MatrixXd cov;
   Eigen::MatrixXd likelihood_mat(val_mat.rows(), val_mat.cols());
   for(int i = 0; i < val_mat.rows(); ++i)
      for(int j = 0; j < val_mat.cols(); ++j){

         if(val_mat(i,j) == 1){
         // get log-likelihoods
         std::tie(mean, cov) = this->trackers_[j].getPredictedMeasStats(this->timestep_);
         double log_likelihood = (meas.col(i) - mean).transpose()*cov.inverse()*(meas.col(i) - mean);

         // construct assoc matrix
         likelihood_mat(i,j) = log_likelihood/this->trackers_[i].getProbabilities()[0];
         }
         else{
         
         likelihood_mat(i,j) = 1e8;

         }
      }
   
   // append no assoc likelihoods
   Eigen::MatrixXd no_assoc_mat = 
      Eigen::MatrixXd::Identity(val_mat.rows(), val_mat.rows())*this->gate_threshold_
      + (Eigen::MatrixXd::Ones(val_mat.rows(), val_mat.rows()) - Eigen::MatrixXd::Identity(val_mat.rows(), val_mat.rows()))*1e8;
   Eigen::MatrixXd assoc_mat(val_mat.rows(), val_mat.cols() + val_mat.rows());
   assoc_mat << likelihood_mat, no_assoc_mat;

   // Find optimal matching
   munkres_cpp::matrix_eigen <double> munkres_input = assoc_mat;
   munkres_cpp::Munkres <double, munkres_cpp::matrix_eigen> solver (munkres_input);

   // Zeros are the assignment places for some absurd reason
   assoc_mat = (1-munkres_input.array()).matrix();

   return assoc_mat;

}

void GnnNode::updateTrackers_(Eigen::MatrixXd assoc_results, Eigen::MatrixXd meas){

   // Find associated mesurements and update trackers
   Eigen::ArrayXi assoc_meas_idx(meas.cols());
   for(int j=0; j<this->trackers_.size(); ++j){

      // If a track has no associations, just propagate in time 
      if(assoc_results.col(j).sum() == 0)
         this->trackers_[j].updateState(this->timestep_);
      else
         for(int i=0; i<meas.cols(); ++i){
            if(assoc_results(i,j) != 0.0){
               this->trackers_[j].updateState(meas.col(i), this->timestep_);
               assoc_meas_idx[i] = i;
            }
         }
      
   }

   // Start tentative tracks with unassociated measurements
   for(int i=0; i<meas.cols(); ++i)
      if((i != assoc_meas_idx).all()){
         int max_id = 0;
         for (int j=0; j<this->trackers_.size(); ++j)
            if(this->trackers_[j].getTrackID() > max_id)
               max_id = this->trackers_[j].getTrackID();
         
         // New tracker has its unique id
         auto tr = ConstantVelocityTracker(meas.col(i), this->process_variance_, this->measurement_variance_, max_id+1);
         this->trackers_.push_back(tr); 
      }

}


// Update elapsed time between  the last two measurement sets
void GnnNode::updateTimestep_(){

   int n = this->last_processed_meas_;

   if (n < 1){
      // Set a naïve default value
      this->timestep_ = 1.0;
   }
   else{
      Eigen::VectorXd time 
         = this->all_measurements_[n].getTimestamp() - this->all_measurements_[n-1].getTimestamp();

      // Convert to seconds
      time(0) = time(0)*3600; // hours to seconds
      time(1) = time(1)*60;   // minutes to seconds
      time(3) = time(3)/1e3;  // milliseconds to seconds

      this->timestep_ = time.sum();
   }
}


void GnnNode::manageTrackers_(){


   // Delete invalid trackers
   this->trackers_.erase(std::remove_if(
      this->trackers_.begin(), this->trackers_.end(), 
      [](auto& tr){return !tr.isValid();}), this->trackers_.end());
   
   // TODO: remove duplicate trackers

}

// Called whenever an input arrives, initiates a measurement set object
void GnnNode::subCallback_(const std_msgs::Float32MultiArrayConstPtr& raw_data){

   this->all_measurements_.push_back(MeasurementSet(raw_data, this->input_length_));

};
