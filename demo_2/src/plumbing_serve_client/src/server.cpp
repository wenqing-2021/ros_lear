#include "ros/ros.h"
#include "plumbing_serve_client/addints.h"
/*
   服务端：解析客户端提交的数据并进行运算
      1. 包含头文件
      2. 初始化ros节点
      3. 创建节点句柄
      4. 处理请求产生响应
      5. spin() 回旋函数
*/

// judge the inputs data
bool doNums(plumbing_serve_client::addints::Request &request,
            plumbing_serve_client::addints::Response &response){
    // case 1:
    int num1 = request.num1; 
    int num2 = request.num2;

    ROS_INFO("receive data is num1= %d, num2= %d", num1, num2);

    int sum = num1 + num2;
    response.sum = sum;

    ROS_INFO("the sum is %d", sum);



    // case 2:
    return true;
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // initial ros node
    ros::init(argc, argv, "serve_sum");
    // creat node handle
    ros::NodeHandle nh;
    // create a server object
    ros::ServiceServer server = nh.advertiseService("addints123", doNums);
    ROS_INFO("server launched");
    ros::spin();
    return 0;
}