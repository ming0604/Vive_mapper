#include "Vive_mapper/grid_map.h"


GridMap::GridMap(double reso, int w, int h):resolution(reso), width(w), height(h)
{
    //origin at the center cell
    origin_x_cells = width/2;
    origin_y_cells = height/2;
    origin_x_meters = origin_x_cells * resolution + 0.5*resolution;
    origin_y_meters = origin_y_cells * resolution + 0.5*resolution;

    // Resize the grid and set all cells to 0 (logodds are 0, means that all cells's probability are 0.5 at beginning)
    cells = std::vector<double>(width * height, 0);
}

GridMap::~GridMap() {}

void GridMap::setCell(int index, double value)
{
    cells[index] = value;
}

int GridMap::world_to_pixel_x(double x_world) const
{   

    return static_cast<int>((x_world + origin_x_meters ) / resolution);
}

int GridMap::world_to_pixel_y(double y_world) const
{   

    return static_cast<int>((y_world + origin_y_meters ) / resolution);
}

int GridMap::pixel_index_to_1Dindex(int x, int y) const
{
    if (x < 0 || x >= width || y < 0 || y >= height) 
    {   
        ROS_ERROR("Error! pixel_index is out of range!" );
        return -1;   // invalid index
    }
    else return x + y * width ;
}

void GridMap::check_and_extend(double x_world, double y_world)
{
    int pixel_x = world_to_pixel_x(x_world);
    int pixel_y = world_to_pixel_y(y_world);

    bool extend_map = false;
    int extend_left = 0, extend_right = 0, extend_bottom = 0, extend_top = 0;
    int new_width, new_height;


    // Check if x coordinate requires map extension
    if (pixel_x < 0) //extend left column
    {
        extend_map = true;
        extend_left = -pixel_x;
        new_width = width + extend_left;
        origin_x_cells += extend_left;
        ROS_INFO("needs left extension: %d columns", extend_left);
    } 
    else if (pixel_x > width-1) //extend right column
    {
        extend_map = true;
        extend_right = pixel_x - (width - 1);
        new_width = width + extend_right;
        ROS_INFO("needs right extension: %d columns", extend_right);
    }

    // Check if y coordinate requires map extension
    if (pixel_y < 0) //extend bottom row
    {
        extend_map = true;
        extend_bottom = -pixel_y;
        new_height = height + extend_bottom;
        origin_y_cells += extend_bottom;
        ROS_INFO("needs bottom extension: %d rows", extend_bottom);
    } 
    else if (pixel_y >= height) //extend top row
    {
        extend_map = true;
        extend_top = pixel_y - (height - 1);
        new_height = height + extend_top;
        ROS_INFO("needs top extension: %d rows", extend_top);
    }

    // If extension is needed, create a new grid and copy old data
    if (extend_map) 
    {   
        ROS_INFO("doing map extension");
        std::vector<double> new_cells(new_width * new_height, 0);

        // Copy old map data to the new map
        int new_pixel_x, new_pixel_y;
        for (int y = 0; y < height; y++) 
        {
            for (int x = 0; x < width; x++) 
            {   
                //only the extension of the left and bottom has different index
                new_pixel_x = x + extend_left;
                new_pixel_y = y + extend_bottom;
                new_cells[new_pixel_x + new_pixel_y * new_width] = cells[pixel_index_to_1Dindex(x, y)];
            }
        }

        // Update grid map properties
        width = new_width;
        height = new_height;
        cells = new_cells;    //if needs the efficiency,trying to use swap() instead of using "="

        // Update origin in meters after extension
        origin_x_meters = origin_x_cells * resolution + 0.5*resolution;
        origin_y_meters = origin_y_cells * resolution + 0.5*resolution;
    }
}

double GridMap::probToLogOdds(double prob) const
{   
    float odds = (prob / (1.0 - prob));
    return log(odds);
}

double GridMap::logOddsToProb(double log_odds) const 
{   
    return exp(log_odds) / (1.0 + exp(log_odds));
}

void GridMap::cellUpdate(int index, double log_update)
{
    cells[index] += log_update;
}

double GridMap::getResolution() const 
{
    return resolution;
}

int GridMap::getWidth() const 
{
    return width;
}

int GridMap::getHeight() const 
{
    return height;
}

void GridMap::getOriginInCell(int& x_result, int& y_result) const
{
    x_result = origin_x_cells;
    y_result = origin_y_cells;
}

void GridMap::getOriginInMeters(double& x_result, double& y_result) const
{
    x_result = origin_x_meters;
    y_result = origin_y_meters;
}

const std::vector<double>& GridMap::getCells() const {
    return cells;
}