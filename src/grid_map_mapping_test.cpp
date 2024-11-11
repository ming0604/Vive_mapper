#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "Vive_mapper/grid_map_mapping.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "grid_map_mapping_test");
    ros::NodeHandle nh;
    // Create a ROS publisher to publish the map to the "map" topic
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    ros::param::set("use_sim_time", false);

    // Create a GridMapMapping object with specified some properties anf parametersㄇ
    double resolution = 0.025;
    int width = 5;
    int height = 9;
    double occ_update_prob = 0.7;  // Occupied update probability
    double free_update_prob = 0.4; // Free update probability
    double occ_threshold = 0.95;    // Occupied threshold probability
    double free_threshold = 0.15;   // Free threshold probability

    GridMapMapping grid_map_mapping(resolution, width, height, occ_update_prob, free_update_prob, occ_threshold, free_threshold);
    map_pub.publish(grid_map_mapping.toOccupancyGrid()); // Publish the original map
    ros::Duration(2.0).sleep();
    // Define publish rate
    ros::Rate loop_rate(1); // Publish once per second
    // Define Lidar starting point
    double x_lidar_world = 0.0;
    double y_lidar_world = 0.0;

    // Define 16 directions
    std::vector<std::pair<double, double>> directions = {
        {0.5, 0.0},   // Right
        {0.5, 0.5},   // Right-Up
        {0.0, 0.5},   // Up
        {-0.5, 0.5},  // Left-Up
        {-0.5, 0.0},  // Left
        {-0.5, -0.5}, // Left-Down
        {0.0, -0.5},  // Down
        {0.5, -0.5},  // Right-Down

        // Additional 8 intermediate directions
        {0.5, 0.25},  // Right-Up slightly Down
        {0.25, 0.5},  // Up slightly Right
        {-0.25, 0.5}, // Up slightly Left
        {-0.5, 0.25}, // Left-Up slightly Down
        {-0.5, -0.25},// Left-Down slightly Up
        {-0.25, -0.5},// Down slightly Left
        {0.25, -0.5}, // Down slightly Right
        {0.5, -0.25}  // Right-Down slightly Up
    };

    int iteration_cnt = 0;
    while (ros::ok())
    {

        for (int i = 0; i < directions.size(); i++)
        {
            double x_end_world = directions[i].first;
            double y_end_world = directions[i].second;

            // Update the map with cells from start to end as free, end point as occupied
            grid_map_mapping.updateMap(x_lidar_world, y_lidar_world, x_end_world, y_end_world);

            // Convert the internal map to a ROS OccupancyGrid message
            nav_msgs::OccupancyGrid ros_map = grid_map_mapping.toOccupancyGrid();

            // Publish the map
            map_pub.publish(ros_map);
            ROS_INFO("Published grid map in direction (%f, %f)", x_end_world, y_end_world);
            ros::spinOnce();
            loop_rate.sleep();
        }
        iteration_cnt++;
        ROS_INFO("Iteration %d completed\n", iteration_cnt);
    }

    return 0;
}