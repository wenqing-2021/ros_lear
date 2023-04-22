#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
/*
   订阅方实现：
   1. 包含头文件： ros/ros.h String
   2. 初始化ros节点
   3. 创建节点句柄
   4. 创建订阅者对象
   5. 处理订阅到的信息
   6. spin函数
*/

void domsg (const std_msgs::String::ConstPtr &msg){
    //通过msg操作订阅到的数据
    // log
    ROS_INFO("输入的信息是 %s",msg->data.c_str());

}
int main(int argc, char* argv[])
{
    setlocale(LC_ALL,"");
    // initial node
    ros::init(argc, argv, "sub_1");
    // create node handle
    ros::NodeHandle nh;
    // subcribers (第一个是topic名称，第二个是queue的长度，第三个是回调函数名称)
    ros::Subscriber sub = nh.subscribe<std_msgs::String>("topic_name_1",10,domsg);
    // 处理订阅数据
    ros::spin();

    return 0;
}