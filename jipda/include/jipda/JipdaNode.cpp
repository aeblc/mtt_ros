#include<jipda/JipdaNode.h>

JipdaNode::JipdaNode() {

    // Gather parameters from the launch file
    this->n_.getParam("/jipda_node/subs_topic", this->subs_topic_);
    this->n_.getParam("/jipda_node/marker_pub_topic", this->marker_pub_topic_);
    this->n_.getParam("/jipda_node/rate", this->rate_);

    ROS_INFO("My node is alive!!");

    this->subs_ = this->n_.subscribe(this->subs_topic_, 10, &JipdaNode::subCallback, this);
    this->marker_pub_ = this->n_.advertise<visualization_msgs::Marker>(this->marker_pub_topic_, 10, this);


};

void JipdaNode::spin()
{
   ros::Rate rate(this->rate_);

   while (ros::ok()){
      ros::spinOnce();

      if (!this->all_measurements_.empty()){
         marker_pub_.publish(this->all_measurements_.back().getMeasPlotter());
      }

      rate.sleep();
      
   }
}


void JipdaNode::subCallback(const std_msgs::Float32MultiArrayConstPtr& raw_data){

   this->all_measurements_.push_back(MeasurementSet(raw_data, 14));

};
