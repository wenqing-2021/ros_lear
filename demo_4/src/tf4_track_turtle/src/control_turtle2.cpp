/*

需求:
    换算turtle1在turtle2中的坐标，并且计算出角速度，速度发布
实现流程:
    1.包含头文件
    2.初始化 ros 节点
    3.创建 ros 句柄
    4.创建 TF 订阅对象
    5.解析订阅信息中获取 son1 坐标系原点在 son2 中的坐标
      解析 son1 中的点相对于 son2 的坐标
    6.spin

*/
//1.包含头文件
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char *argv[])
{   
    setlocale(LC_ALL,"");
    // 2.初始化 ros 节点
    ros::init(argc,argv,"sub_frames");
    // 3.创建 ros 句柄
    ros::NodeHandle nh;
    // 4.1 创建 TF 订阅对象
    tf2_ros::Buffer buffer; 
    tf2_ros::TransformListener listener(buffer);
    // 4.2 创建对象，发布turtle2的线速度角速度
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 100);


    // 5.解析订阅信息中获取 turtle1 坐标系原点在 turtle2 中的坐标
    ros::Rate r(1);
    while (ros::ok())
    {
        try
        {
        //   解析 son1 中的点相对于 son2 的坐标, son2是目标坐标系
            geometry_msgs::TransformStamped tfs = buffer.lookupTransform("turtle2","turtle1",ros::Time(0)); // 使用lookupTransform在多坐标系中变换, 参数1是父坐标（参考的），参数2是子坐标
            // ROS_INFO("turtle1 相对于 turtle2 的坐标关系: 父坐标系ID=%s, 自坐标系ID=%s, 坐标偏移量(x=%.2f,y=%.2f,z=%.2f)",
            //           tfs.header.frame_id.c_str(),
            //           tfs.child_frame_id.c_str(),
            //           tfs.transform.translation.x,
            //           tfs.transform.translation.y,
            //           tfs.transform.translation.z); // son1原点与son2原点的偏移量
            // 计算并组织消息
            geometry_msgs::Twist twist;
            // 此处的0.5 和 4是线速度和角速度的系数
            twist.linear.x = 0.5 * sqrt(pow(tfs.transform.translation.x,2) + pow(tfs.transform.translation.y,2)); // 这里的x方向线速度是turtle2自身坐标系下的，即x方向为航向角
            twist.angular.z = 1.0 * atan2(tfs.transform.translation.y,tfs.transform.translation.x);// z方向的角速度，即yaw

            // 发布
            pub.publish(twist);
        }
        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            ROS_INFO("异常信息:%s",e.what());
        }


        r.sleep();
        // 6.spin
        ros::spinOnce();
    }
    return 0;
}