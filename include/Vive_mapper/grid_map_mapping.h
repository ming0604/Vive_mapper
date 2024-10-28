#ifndef GRID_MAP_MAPPING_H
#define GRID_MAP_MAPPING_H


#include "grid_map.h"
#include <nav_msgs/OccupancyGrid.h>
#include <cmath>


class GridMapMapping
{
    public:
        GridMapMapping(double resolution, int width, int height, double occ_update_prob, double free_update_prob, double occ_threshold, double free_threshold);
        virtual ~GridMapMapping();
        void BresenhamLineUpdate(int x0, int y0, int x1, int y1);    // Helper function to trace the ray using Bresenham's line algorithm
        void updateMap(double x_lidar_world, double y_lidar_world, double x_end_world, double y_end_world); // Update the grid map based on the ray traced by Bresenham's line algorithm
        nav_msgs::OccupancyGrid toOccupancyGrid() const;// Convert internal grid map to ROS OccupancyGrid for publishing
    private:
        double occ_thres_prob;
        double free_thres_prob;
        double occ_thres_logodds;
        double free_thres_logodds;
        double logocc;
        double logfree;
        GridMap map;
};

#endif