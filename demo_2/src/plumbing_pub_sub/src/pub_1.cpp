#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
/*
   发布方实现：
   1. 包含头文件： ros/ros.h String
   2. 初始化ros节点
   3. 创建节点句柄
   4. 创建发布者对象
   5. 编写发布逻辑
*/
int main(int argc, char *argv[])
{
    setlocale(LC_ALL,""); // 如果要输出中文，必须有这个
    // initial ros node
    ros::init(argc,argv,"pub_1");
    // create node handle
    ros::NodeHandle nh;
    // publisher
    // advertise<数据类型>("话题名"， 数据长度)
    ros::Publisher pub = nh.advertise<std_msgs::String>("topic_name_1",10);
    // publish data
    // create mesage object
    std_msgs::String msg;
    // set publish frequency
    ros::Rate rate(10);
    // ros::ok() 如果节点存在，循环就成立
    int count = 0;

    ros::Duration(3).sleep();
    while(ros::ok())
    {
        count++;
        // contact num and string
        std::stringstream ss;
        ss << "hello--->" << count; // 这个里面不能有中文，哈哈哈
        msg.data = ss.str();
        pub.publish(msg);
        // log
        ROS_INFO("输出的信息是 %s", msg.data.c_str());
        //根据前面制定的发送贫频率自动休眠 休眠时间 = 1/频率；
        rate.sleep();
    }

    return 0;
}