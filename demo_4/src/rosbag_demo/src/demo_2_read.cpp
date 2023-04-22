#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
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
    bag.open("hello.bag", rosbag::BagMode::Read); // 打开文件流

    // 获取消息的集合，再迭代取出消息的字段
    for (auto &&i : rosbag::View(bag))
    {
        // 解析，获取话题，时间戳，消息
        std::string topic = i.getTopic();
        ros::Time time = i.getTime();
        std_msgs::StringConstPtr p = i.instantiate<std_msgs::String>();
        ROS_INFO("话题内容：%s, 时间戳：%.2f, 消息值：%s",
                topic.c_str(),
                time.toSec(),
                p->data.c_str());
    }
    bag.close(); // 关闭文件流
    ROS_INFO("已经读取数据");
    return 0;
}
