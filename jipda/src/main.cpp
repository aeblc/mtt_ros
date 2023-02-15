#include<jipda/JipdaNode.h>


int main(int argc, char** argv){

    ros::init(argc, argv, "jpda_node");

    JipdaNode node;

    node.spin(); 

    return 0;
}