#include"ros/ros.h"

/* 
   参数的新增与修改
 */

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "param_node");
    ros::NodeHandle nh;
    // add parameters
    nh.setParam("type","yellow_car");
    nh.setParam("size", 2);
    ros::param::set("height_set",3.15);
    ros::param::set("radius", 1.1);
    ROS_INFO("have set para  meters");

    // 
    return 0;
}
