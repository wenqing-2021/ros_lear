#include "use_head/head.h"
#include "ros/ros.h"

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"hahah");
    hello_ns::HelloPub hello;
    hello.run();
    return 0;
}