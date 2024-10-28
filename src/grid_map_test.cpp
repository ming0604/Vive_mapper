#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <iostream>
#include "Vive_mapper/grid_map.h"


// Print the current grid map
void printGrid(const GridMap& map) {
    int width = map.getWidth();
    int height = map.getHeight();
    const std::vector<double>& cells = map.getCells();

    std::cout << "Grid (" << width << "x" << height << ")" << std::endl;
    for (int y = height-1; y >= 0; y--) 
    {
        for (int x = 0; x < width; x++) 
        {
            double value = cells[map.pixel_index_to_1Dindex(x, y)];
            std::cout << value << " ";  
        }
        std::cout << std::endl;
    }
}

// Convert the internal grid map into a ROS OccupancyGrid message
nav_msgs::OccupancyGrid toOccupancyGrid(const GridMap& map, double free_prob, double occ_prob) {
    nav_msgs::OccupancyGrid ros_map;
    // Set the metadata for the map
    ros_map.info.resolution = map.getResolution();
    ros_map.info.width = map.getWidth();
    ros_map.info.height = map.getHeight();

    // Set the origin of the map in world coordinates
    double origin_x, origin_y;
    map.getOriginInMeters(origin_x, origin_y);
    ros_map.info.origin.position.x = -origin_x;
    ros_map.info.origin.position.y = -origin_y;
    ros_map.info.origin.position.z = 0.0;
    ROS_INFO("origin_x: %f, origin_y: %f", ros_map.info.origin.position.x, ros_map.info.origin.position.y);
    // Default quaternion is identity(no rotation)

    // Convert log-odds to probability [0, 100] and populate the grid data
    ros_map.data.resize(map.getWidth() * map.getHeight());
    const std::vector<double>& cells = map.getCells();
    for (size_t i = 0; i < cells.size(); i++) {
        double prob = map.logOddsToProb(cells[i]);

        //use logodds to determine the state may be more efficient
        if(prob<=free_prob)
        {
            ros_map.data[i] = 0;
        }
        else if(prob>=occ_prob)
        {
            ros_map.data[i] = 100;
        }
        else
        {
            ros_map.data[i] = -1;
        }
    }
    ros_map.header.frame_id = "map";
    ros_map.header.stamp = ros::Time::now();
    return ros_map;
}

int main(int argc, char* argv[]) {
    // Initialize ROS node
    ros::init(argc, argv, "grid_map_test");
    ros::NodeHandle nh;

    // Create a 5x9 grid map with a resolution of 0.1 meters per cell
    GridMap map(0.05, 5, 9);
    double free_prob = 0.49;
    double occ_prob = 0.51;
    // Create a ROS publisher for the map topic
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 2,true);

    ros::Rate loop_rate(0.1);  // Publish once every 10 second (0.1hz)

    while (ros::ok()) {
        // Print the current state of the grid map
        std::cout << "Current Grid:" << std::endl;
        printGrid(map);
        nav_msgs::OccupancyGrid ros_map_before = toOccupancyGrid(map,free_prob,occ_prob);
        map_pub.publish(ros_map_before);
        ROS_INFO("Published map before changes");
        ros::Duration(2.0).sleep();
        
        // Test normal coordinate conversion and cell update
        double x_world = 0.01;
        double y_world = 0.01;
        int pixel_x, pixel_y, index;
        //check map extension with the given world coordinates
        map.check_and_extend(x_world, y_world);
        pixel_x = map.world_to_pixel_x(x_world);
        pixel_y = map.world_to_pixel_y(y_world);
        index = map.pixel_index_to_1Dindex(pixel_x, pixel_y);

        // If the index is valid, update the cell with a log-odds value
        if (index != -1) {
            map.cellUpdate(index, -2);
            ROS_INFO("Updated cell at (%d, %d)", pixel_x, pixel_y);
        }

        // Convert the internal grid map to an OccupancyGrid and publish it
        nav_msgs::OccupancyGrid ros_map_changed = toOccupancyGrid(map,free_prob,occ_prob);
        map_pub.publish(ros_map_changed);
        ROS_INFO("Published map after normal update");
        ros::Duration(2.0).sleep();

        //test after top-right extension
        x_world = 0.5;
        y_world = 0.5;
        map.check_and_extend(x_world, y_world);
        pixel_x = map.world_to_pixel_x(x_world);
        pixel_y = map.world_to_pixel_y(y_world);
        index = map.pixel_index_to_1Dindex(pixel_x, pixel_y);

        // If the index is valid, update the cell with a log-odds value
        if (index != -1) {
            map.cellUpdate(index, 2);
            ROS_INFO("Updated cell at (%d, %d)", pixel_x, pixel_y);
        }
        // Convert the internal grid map to an OccupancyGrid and publish it
        nav_msgs::OccupancyGrid ros_map_extended_top_right = toOccupancyGrid(map,free_prob,occ_prob);
        map_pub.publish(ros_map_extended_top_right);
        ROS_INFO("Published map after extended top-right");
        ros::Duration(2.0).sleep();

        //test after bottom-left extension
        x_world = -0.5;
        y_world = -0.5;
        map.check_and_extend(x_world, y_world);
        pixel_x = map.world_to_pixel_x(x_world);
        pixel_y = map.world_to_pixel_y(y_world);
        index = map.pixel_index_to_1Dindex(pixel_x, pixel_y);

        // If the index is valid, update the cell with a log-odds value
        if (index != -1) {
            map.cellUpdate(index, 2);
            ROS_INFO("Updated cell at (%d, %d)", pixel_x, pixel_y);
        }
        // Convert the internal grid map to an OccupancyGrid and publish it
        nav_msgs::OccupancyGrid ros_map_extended_bottom_left = toOccupancyGrid(map,free_prob,occ_prob);
        map_pub.publish(ros_map_extended_bottom_left);
        ROS_INFO("Published map after extended bottom-left");
        ros::Duration(2.0).sleep();
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}