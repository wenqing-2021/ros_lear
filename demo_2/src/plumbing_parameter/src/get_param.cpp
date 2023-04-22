#include"ros/ros.h"

/*
   参数查询同样有两套API, ros::NodeHandle 以及ros::param

   ros::NodeHandle:
        param(键,默认值) 
            存在，返回对应结果，否则返回默认值

        getParam(键,存储结果的变量)
            存在,返回 true,且将值赋值给参数2
            若果键不存在，那么返回值为 false，且不为参数2赋值

        getParamCached(键,存储结果的变量)--提高变量获取效率
            存在,返回 true,且将值赋值给参数2
            若果键不存在，那么返回值为 false，且不为参数2赋值

        getParamNames(std::vector<std::string>)
        e.g. std::vector<std::string> names;
             nh.getParaNames(names);
            获取所有的键,并存储在参数 vector 中 

        hasParam(键)
            是否包含某个键，存在返回 true，否则返回 false

        searchParam(参数1，参数2)
        e.g. std::string key; 
             nh.searchParam("radius", key);
            搜索键，参数1是被搜索的键，参数2存储搜索结果的变量,将参数变量存在参数2
    
    ros::param
        param("radius", 100.5);
*/



int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");

    ros::init(argc,argv,"get_param");
    ros::NodeHandle nh;

    double radius = nh.param("radius", 0.5); //查询radius，如果不存在就返回0.5
    ROS_INFO("radius = %.2f", radius);
    std::string key; 
    nh.searchParam("radius", key);
    ROS_INFO("result:%s", key.c_str());

    double radius_param = ros::param::param("radius", 100.5);
    return 0;
}
