#ifndef GRID_MAP_H
#define GRID_MAP_H

#include <vector>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

class GridMap 
{
    public:
        GridMap(double reso, int w, int h);
        virtual ~GridMap();

        void setCell(int index, double value);
        int world_to_pixel_x(double x_world) const;
        int world_to_pixel_y(double y_world) const;
        int pixel_index_to_1Dindex(int x, int y) const;
        void check_and_extend(double x_world, double y_world);

        double probToLogOdds(double prob) const;
        double logOddsToProb(double log_odds) const;
        void cellUpdate(int index, double log_update);
        
        double getResolution() const;
        int getWidth() const;
        int getHeight() const;
        void getOriginInCell(int& x_result, int& y_result) const;
        void getOriginInMeters(double& x_result, double& y_result) const;
        const std::vector<double>& getCells() const;

    private:
        double resolution;      //  meters/cell
        int width;              //  in cells
        int height;             //  in cells
        int origin_x_cells, origin_y_cells; // Origin relative to the bottom-left corner (in cells)
        double origin_x_meters, origin_y_meters; // Origin relative to the bottom-left corner (in meters)

        std::vector<double> cells;  // logodds for cells
};


#endif
