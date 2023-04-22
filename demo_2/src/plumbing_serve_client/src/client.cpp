# include "ros/ros.h"
#include "plumbing_serve_client/addints.h"
/* 客户端：提交两个整数，并处理响应结果
     1. 头文件
     2. 初始化节点
     3. 创建节点句柄
     4. 创建客户端对象
     5. 提交响应并处理

    实现参数的动态提交：
      1. 格式： rosrun xxxx xxxx 12 34
      2. 节点执行时，需要获取命令中的参数，并组织近request里
    
    实际问题：
      1. 需求为先启动客户端，等待服务器启动
 */

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // input two int
    if (argc != 3){ // 三个参数，文件名，参数1，参数2
        ROS_INFO("wrong input number");
        return 1;
    }
    ros::init(argc,argv,"client");
    ros::NodeHandle nh;
    ros::ServiceClient client=nh.serviceClient<plumbing_serve_client::addints>("addints123"); // 选择server中的节点名称

    // send a request
    plumbing_serve_client::addints ai;
    ai.request.num1 = atoi(argv[1]); // 将字符串变成int类型数据
    ai.request.num2 = atoi(argv[2]);

    // 处理响应
    // 方法一：
    // client.waitForExistence(); // 判断服务器状态，等待服务器启动后再进行响应
    // 方法二：
    ros::service::waitForService("addints123"); // 效果和方法一样，不过需要传参，括号里面的选择server中的节点名称
    bool flag = client.call(ai);
    if (flag)
    {
        ROS_INFO("successfully response");
        ROS_INFO("the result is %d", ai.response.sum);
    } else{
        ROS_INFO("failed to response");
    }

    return 0;
}