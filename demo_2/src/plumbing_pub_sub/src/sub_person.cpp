
#include "ros/ros.h"
#include "plumbing_pub_sub/person.h"

void domsg(const plumbing_pub_sub::person::ConstPtr &msg){
     ROS_INFO("subscriber data is:%s, %d, %.2f", msg->name.c_str(), msg->age, msg->height);
}

int main(int argc, char *argv[]){
    setlocale(LC_ALL,"");

    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("chat", 10, domsg);

    ros::spin();

    return 0;
}