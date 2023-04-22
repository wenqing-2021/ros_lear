#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

/* 动态实时发布乌龟相对于窗体的坐标关系
    准备：
       获取乌龟位姿话题： /turtle1/pose
       以及消息类型：/turtlesim/Pose
    实现流程：
       1. 头文件
       2. 编码、初始化、Nodehandle
       3. 创建订阅对象，订阅话题；
       4. 回调函数，处理订阅信息转换为坐标相对关系并发布
       5. spin()

 */

std::string turtle_name;

void doPose(const turtlesim::Pose::ConstPtr& pose){
    // a. 创建发布对象
    static tf2_ros::TransformBroadcaster pub;
    // b. 组织被发布的数据
    geometry_msgs::TransformStamped ts; // 发布数据放在ts中
    ts.header.frame_id = "world_frame";
    ts.header.stamp = ros::Time::now();

    // 此处需要切换小乌龟坐标名称
    ts.child_frame_id = turtle_name;
    // 坐标系偏移量设置
    ts.transform.translation.x = pose->x;
    ts.transform.translation.y = pose->y;
    ts.transform.translation.z = 0;
    //坐标四元数设置
    tf2::Quaternion qtn;
    qtn.setRPY(0,0,pose->theta);
    ts.transform.rotation.x = qtn.getX();
    ts.transform.rotation.y = qtn.getY();
    ts.transform.rotation.z = qtn.getZ();
    ts.transform.rotation.w = qtn.getW();
    // c. 发布
    pub.sendTransform(ts);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "pub_pose");
    ros::NodeHandle nh;

     // 解析args传入的参数
    if (argc !=2 )
    {
        ROS_ERROR("请传入正确的参数");
        return 1;
    }else{
        turtle_name = argv[1];
        ROS_INFO("乌龟 %s 坐标发送启动",turtle_name.c_str());
    }

    // 注意!此处的订阅对象也必须修改
    ros::Subscriber sub = nh.subscribe<turtlesim::Pose>(turtle_name + "/pose",100,doPose); // 此处的doPose是回调函数

    ros::spin();

    return 0;
}
