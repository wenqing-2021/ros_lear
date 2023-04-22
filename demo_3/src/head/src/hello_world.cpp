#include "ros/ros.h"
#include "head/head.h"

namespace hello_ns {

void HelloPub::run(){
    ROS_INFO("hello_world");
}
}

int main(int argc, char *argv[])
{
    /* code */
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"head_node"); // 创建节点
    hello_ns::HelloPub helloPub; // 实例化class
    helloPub.run();
    return 0;
}
