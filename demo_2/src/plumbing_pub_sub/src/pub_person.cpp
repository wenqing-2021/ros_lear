#include "ros/ros.h"
#include "plumbing_pub_sub/person.h"

int main(int argc, char *argv[]){
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "person_pub");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<plumbing_pub_sub::person>("chat", 10);
    // 创建发布的数据
    plumbing_pub_sub::person person;
    person.name = "yuansj";
    person.age = 22;
    person.height = 1.83;

    ros::Rate rate(1);

    ros::Duration(3).sleep();
    while (ros::ok())
    {
        // 修改被发布的数据
        person.age += 1;
        // 发布数据
        pub.publish(person);
        ROS_INFO("publisher data is %s, %d, %.2f", person.name.c_str(), person.age, person.height);
        // 休眠
        rate.sleep();
        // 回调函数
        ros::spinOnce();
    }
    

    return 0;
}