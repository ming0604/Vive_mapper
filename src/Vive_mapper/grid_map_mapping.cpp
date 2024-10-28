#include "Vive_mapper/grid_map_mapping.h"

// Constructor: initialize GridMap and log-odds values
GridMapMapping::GridMapMapping(double resolution, int width, int height, double occ_update_prob, double free_update_prob, double occ_threshold, double free_threshold):
 map(resolution, width, height), occ_thres_prob(occ_threshold), free_thres_prob(free_threshold)
{
    logocc = map.probToLogOdds(occ_update_prob);
    logfree = map.probToLogOdds(free_update_prob);
    occ_thres_logodds = map.probToLogOdds(occ_thres_prob);
    free_thres_logodds = map.probToLogOdds(free_thres_prob);
}

GridMapMapping::~GridMapMapping() {}


void GridMapMapping::BresenhamLineUpdate(int x0, int y0, int x1, int y1)
{
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);

    int err;
    // Mark startpoint as free
    map.cellUpdate(map.pixel_index_to_1Dindex(x0, y0), logfree);
    // Determine the main direction
    int x,y,x_start,y_start,x_end,y_end;
    int dir;
    if (dx >= dy) // x as the main step
    {   
        // if x1<x0 then swap the start and end points, ensure the line always grows to the right
        if(x1 < x0)  
        {
            x_start = x1;     
            x_end = x0;
            y_start = y1;    
            y_end = y0;
        }
        else
        {
            x_start = x0;    
            x_end = x1;
            y_start = y0;     
            y_end = y1;
        }
        // if y_end<y_start , y go decreasing direction ,dir=-1
        if(y_end < y_start)
        {   
            dir = -1;
        }
        else {dir = 1;}

        err = 2 * dy - dx;
        x = x_start;
        y = y_start;
        while (x < x_end-1) 
        {
            if (err >= 0) 
            {
                y += dir;
                err -= 2 * dx;
            }
            x += 1;
            err += 2 * dy;
            map.cellUpdate(map.pixel_index_to_1Dindex(x, y), logfree);  // Update cell as free space
        }
    } 
    else 
    {   // y as the main step
        
        // if y1<y0 then swap the start and end points, ensure the line always grows upward
        if(y1 < y0)  
        {
            x_start = x1;     //x_start
            x_end = x0;
            y_start = y1;     //y_start
            y_end = y0;
        }
        else
        {
            x_start = x0;     //x_start
            x_end = x1;
            y_start = y0;     //y_start
            y_end = y1;
        }
        // if x_end<x_start , x go decreasing direction ,dir=-1
        if(x_end < x_start)
        {   
            dir = -1;
        }
        else {dir = 1;}

        err = 2 * dx - dy;
        x = x_start;
        y = y_start;
        while (y < y_end-1) 
        {
            if (err >= 0) 
            {
                x += dir;
                err -= 2 * dy;
            }
            y += 1;
            err += 2 * dx;
            map.cellUpdate(map.pixel_index_to_1Dindex(x, y), logfree);  // Update cell as free space
        }
    } 
    
    map.cellUpdate(map.pixel_index_to_1Dindex(x1, y1), logocc); // Mark endpoint as occupied

}

// Updates map based on LiDAR start and end points in world coordinates
void GridMapMapping::updateMap(double x_lidar_world, double y_lidar_world, double x_end_world, double y_end_world)
{
    // Check and extend map if end point is out of current bounds
    map.check_and_extend(x_lidar_world, y_lidar_world);
    map.check_and_extend(x_end_world, y_end_world);
    
    // Convert world coordinates to grid cell indices
    int x0 = map.world_to_pixel_x(x_lidar_world);
    int y0 = map.world_to_pixel_y(y_lidar_world);
    int x1 = map.world_to_pixel_x(x_end_world);
    int y1 = map.world_to_pixel_y(y_end_world);

    // Update cells along the line from start to end
    BresenhamLineUpdate(x0, y0, x1, y1);
}

nav_msgs::OccupancyGrid GridMapMapping::toOccupancyGrid() const
{   
    // Set map metadata
    nav_msgs::OccupancyGrid ros_map;
    ros_map.info.resolution = map.getResolution();
    ros_map.info.width = map.getWidth();
    ros_map.info.height = map.getHeight();

    // Set the origin of the map (bottom-left corner) in world coordinates
    double origin_x, origin_y;
    map.getOriginInMeters(origin_x, origin_y);
    ros_map.info.origin.position.x = -origin_x;
    ros_map.info.origin.position.y = -origin_y;
    ros_map.info.origin.position.z = 0.0;
    ROS_INFO("ros grid map origin_x: %f, origin_y: %f", ros_map.info.origin.position.x, ros_map.info.origin.position.y);
    // Default quaternion is identity(no rotation)

    // Convert log-odds to probability [0, 100] and populate the grid data
    ros_map.data.resize(map.getWidth() * map.getHeight());
    const std::vector<double>& cells = map.getCells();
    for (size_t i = 0; i < cells.size(); i++) {
        
        // Map log-odds to occupancy grid values (0 for free, -1 for unknown, 100 for occupied)
        if (cells[i] >= occ_thres_logodds) 
        {
            ros_map.data[i] = 100;
        } 
        else if (cells[i] <= free_thres_logodds) 
        {
            ros_map.data[i] = 0;
        } 
        else {
            ros_map.data[i] = -1;
        }
    }

    ros_map.header.frame_id = "map";
    ros_map.header.stamp = ros::Time::now();
    
    return ros_map;
}