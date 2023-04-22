#include "ros/ros.h"
#include "rosbag/bag.h"
#include "std_msgs/String.h"
/* 利用rosbag向磁盘文件写入数据（话题 + 消息）
   流程：
      1. 导包
      2. 初始化
      3. 创建rosbag对象
      4. 打开文件流
      5. 写入数据
      6. 关闭文件流
 */

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "bag_write");
    ros::NodeHandle nh;
    rosbag::Bag bag;
    bag.open("hello.bag", rosbag::BagMode::Write); // 打开文件流
    std_msgs::String msg;
    msg.data = "hello";
    // 参数一：话题名，参数二：时间戳，参数三：消息
    bag.write("/chatter", ros::Time::now(), msg); 
    bag.close(); // 关闭文件流
    ROS_INFO("已经保存数据");
    return 0;
}
