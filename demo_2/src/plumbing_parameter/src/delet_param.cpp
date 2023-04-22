#include"ros/ros.h"

/*  ros::NodeHandle
       delParam()
    ros::param
       del()
 */

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"delet_param");
    ros::NodeHandle nh;
    bool has_delet = nh.deleteParam("radius");
    if (has_delet)
    {
        ROS_INFO("has deleted");
    }else{
        ROS_INFO("Fail deleted");
    }
    return 0;
}
