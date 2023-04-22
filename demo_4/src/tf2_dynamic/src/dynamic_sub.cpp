#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" //注意: 调用 transform 必须包含该头文件
/* 实现流程:
        1.包含头文件
        2.初始化 ROS 节点
        3.创建 TF 订阅节点
        4.生成一个坐标点(相对于子级坐标系)
        5.转换坐标点(相对于父级坐标系)
        6.spin() 
*/

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "static_sub");
    ros::NodeHandle nh;
    // 创建buffer
    tf2_ros::Buffer buffer;
    // 创建监听对象
    tf2_ros::TransformListener listener(buffer);
    // 组织一个坐标点数据
    geometry_msgs::PointStamped ps;
    // 参考坐标系
    ps.header.frame_id = "turtle1";
    // 时间戳
    // 动态数据，每一个数据都有一个时间戳，ros会将坐标点的时间戳和坐标系的时间戳比对，如果差太多了就不行
    // 如果是ros::Time::now() 就会报异常，而下面的写法则是没有规定时间戳的值，所以不会有异常
    ps.header.stamp = ros::Time(0.0); 
    ps.point.x = 2.0;
    ps.point.y = 3.0;
    ps.point.z = 5.0;
    // 添加休眠
    // ros::Duration(2).sleep();
    // 转换算法
    ros::Rate rate(10);
    while (ros::ok())
    {
        // 将ps转换为baselinek的坐标点
         //--------------使用 try 语句或休眠，否则可能由于缓存接收延迟而导致坐标转换失败------------------------
        try
        {
            geometry_msgs::PointStamped ps_out;
            ps_out = buffer.transform(ps,"world_frame");
            ROS_INFO("转换后的数据:(%.2f,%.2f,%.2f),参考的坐标系是:%s",
                                  ps_out.point.x,
                                  ps_out.point.y,
                                  ps_out.point.z,
                                  ps_out.header.frame_id.c_str());

        }
        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            ROS_INFO("程序异常.....");
        }
        
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
