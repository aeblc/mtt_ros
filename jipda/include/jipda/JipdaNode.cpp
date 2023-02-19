#include<jipda/JipdaNode.h>

JipdaNode::JipdaNode() {

   // Gather parameters from the launch file
   this->n_.getParam("/jipda_node/subs_topic", this->subs_topic_);
   this->n_.getParam("/jipda_node/rate", this->rate_);
   this->n_.getParam("/jipda_node/meas_length", this->measurement_length_);
   this->n_.getParam("/jipda_node/proc_var", this->process_variance_);
   this->n_.getParam("/jipda_node/meas_var", this->measurement_variance_);

   ROS_INFO("My node is alive!!");

   this->subs_ = this->n_.subscribe(this->subs_topic_, 10, &JipdaNode::subCallback, this);
   this->meas_marker_pub_ = this->n_.advertise<visualization_msgs::Marker>("/jipda/measurements", 10, this);
   this->state_marker_pub_ = this->n_.advertise<visualization_msgs::Marker>("/jipda/estimated_state", 10, this);
   this->innovation_marker_pub_ = this->n_.advertise<visualization_msgs::Marker>("/jipda/innovation_cov", 10, this);

   

};

void JipdaNode::spin()
{
   ros::Rate rate(this->rate_);
   bool flag = true;

   while (ros::ok()){
      ros::spinOnce();

      if (!this->all_measurements_.empty()){
         this->meas_marker_pub_.publish(this->all_measurements_.back().getMeasPlotter());
         
         if (flag){
            auto meas = this->all_measurements_.back().getParsedOutput().col(0);
            this->trackers_.push_back(ConstantVelocityTracker(meas, this->process_variance_, this->measurement_variance_, 0));
            flag = false;
         }
      }

      if (!this->trackers_.empty()){
         auto meas = this->all_measurements_.back().getParsedOutput().col(0);
         trackers_.back().updateState(meas, 1.0);
         this->state_marker_pub_.publish(trackers_.back().getStateMarker());
         this->innovation_marker_pub_.publish(trackers_.back().getInnovationMarker());
      }

      rate.sleep();
      
   }
}


void JipdaNode::subCallback(const std_msgs::Float32MultiArrayConstPtr& raw_data){

   this->all_measurements_.push_back(MeasurementSet(raw_data, this->measurement_length_));

};
