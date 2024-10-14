#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "Vive_mapper/grid_map.h"



int main(int argc, char* argv[])
{
    ros::init(argc, argv, "grid_map_test_publisher");
    ros::NodeHandle nh;

    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1);
    ros::Rate loop_rate(1); //1hz

    /**/
    nav_msgs::OccupancyGrid map;
    map.header.frame_id = "map";
    map.info.resolution = 0.05;
    map.info.width = 5;
    map.info.height = 8;
    map.info.origin.position.x = -0.1;
    map.info.origin.position.y = -0.2;
    //map.info.origin.position.x = 0;
    //map.info.origin.position.y = 0;
    map.info.origin.position.z = 0;

   // 初始化地圖數據（100 個 cell，初始化為 -1，表示未知）
    map.data.resize(map.info.width * map.info.height);
    std::fill(map.data.begin(), map.data.end(), -1);

    
    for (int i = 0; i < map.info.width * map.info.height; i++) 
    {   
        
        // 偶數格設100障礙物，奇數格設0空閒
        if (i % 2 == 0)
        {
            map.data[i] = 100;  // 佔據的區域（障礙物）
        }
        else 
        {
            map.data[i] = 0;    // 空閒區域
        }
        
        /*
        //only cell 0 set to 100
        if(i==0)
        {
            map.data[i] = 100;  
        }
        else 
        {
            map.data[i] = 0;
        }  
        */   
    }

    // 發佈地圖
    while (ros::ok()) 
    {
        map.header.stamp = ros::Time::now();  // 更新時間戳
        map_pub.publish(map);                 // 發佈地圖消息
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}