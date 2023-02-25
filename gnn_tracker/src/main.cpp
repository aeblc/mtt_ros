#include<gnn/GnnNode.h>


int main(int argc, char** argv){

    ros::init(argc, argv, "gnn_node");

    GnnNode node;

    node.spin(); 

    return 0;
}