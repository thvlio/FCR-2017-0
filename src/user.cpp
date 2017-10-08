#include "fcr2017/obstacle_avoidance.h"

int main (int argc, char** argv) {
    
	ros::init (argc, argv, "user");
    ros::NodeHandle n;
    ros::Publisher position_pub = n.advertise<geometry_msgs::Point> ("user_pos", 10);
    
    ros::Rate loop_rate (50);
    
    while (ros::ok()) {
        
        geometry_msgs::Point userPos;
        std::cout << "Entre com a posição alvo:\n>>";
        std::cin >> userPos.x >> userPos.y >> userPos.z;
        position_pub.publish (userPos);
        
        ros::spinOnce();
        loop_rate.sleep();
        
    }
    
    return 0;
}