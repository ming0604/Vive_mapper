#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2_ros/transform_listener.h>
#include "tf2/LinearMath/Transform.h"
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/utils.h>

#include <nav_msgs/OccupancyGrid.h>
#include "Vive_mapper/grid_map_mapping.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory> 

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> MySyncPolicy;
using namespace std;
class ViveMapperNode
{
    public:
        ViveMapperNode();
        virtual ~ViveMapperNode();
        void syncCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg, const nav_msgs::Odometry::ConstPtr& base_gt_odom_msg);

        tf2::Transform createtf2_from_XYyaw(double x, double y, double theta);
        void tf2_listener_laser();
        void get_laser_pose();
        pcl::PointCloud<pcl::PointXYZ>::Ptr create_scan_pc();
        void update_map(const ros::Time& scan_time);
        bool isStationary(double current_x, double current_y, double current_yaw) ; 

    private:
        // ROS NodeHandle, publishers, and subscribers
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        ros::Publisher map_pub_;
        ros::Publisher scan_pc_pub_;
        message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub_;
        message_filters::Subscriber<nav_msgs::Odometry> base_gt_odom_sub_;
        message_filters::Synchronizer<MySyncPolicy> sync_;
        
        // Map and pose variables
        std::unique_ptr<GridMapMapping> grid_map_mapping_ptr;  //用智能指針,才能在讀到參數後做初始化
        double base_x_, base_y_, base_yaw_;
        double laser_x_, laser_y_, laser_yaw_;
        double last_x_, last_y_, last_yaw_;

        //LiDAR scan parameters
        float laser_angle_min_;
        float laser_angle_max_;
        float laser_angle_increment_;
        float laser_time_increment_;
        float range_max_;
        float range_min_;
        vector<float> ranges_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr scan_pc_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr scan_pc_on_map_;

        //tf2,transform
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        tf2::Transform laser_to_base_tf2_;
        tf2::Transform base_to_map_tf2_;
        tf2::Transform laser_to_map_tf2_;
        geometry_msgs::TransformStamped laser_to_base_geoTf_;
        geometry_msgs::Transform laser_to_map_geoTf_;
        Eigen::Isometry3d laser_to_map_eigen_;

        // Parameters
        double resolution_;
        int width_, height_;
        double occ_update_prob_, free_update_prob_;
        double occ_threshold_, free_threshold_;
        std::string base_frame_id_;
        std::string laser_frame_id_;
        double stationary_d_threshold_;
        double stationary_a_threshold_;

        //other signals
        bool laser_to_base_received_;
};


ViveMapperNode::ViveMapperNode():nh_(), private_nh_("~"), tf_listener_(tf_buffer_), sync_(MySyncPolicy(10), scan_sub_, base_gt_odom_sub_)
{
    // Load parameters
    private_nh_.param("resolution", resolution_, 0.025);
    private_nh_.param("width", width_, 1000);
    private_nh_.param("height", height_, 1000);
    private_nh_.param("occ_update_prob", occ_update_prob_, 0.9);
    private_nh_.param("free_update_prob", free_update_prob_, 0.1);
    private_nh_.param("occ_threshold", occ_threshold_, 0.7);
    private_nh_.param("free_threshold", free_threshold_, 0.3);
    private_nh_.param("stationary_distance_threshold", stationary_d_threshold_, 0.0125);
    private_nh_.param("stationary_angle_threshold", stationary_a_threshold_, (M_PI/180));
    private_nh_.param("range_used_max", range_max_, 20.0f);
    private_nh_.param<string>("base_frame_id", base_frame_id_ ,"base_link");
    private_nh_.param<string>("Lidar_frame_id", laser_frame_id_ ,"laser");
    scan_pc_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    scan_pc_on_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);

    // Initialize GridMapMapping with provided parameters
    //grid_map_mapping_ = GridMapMapping(resolution_, width_, height_, occ_update_prob_, free_update_prob_, occ_threshold_, free_threshold_);
    grid_map_mapping_ptr = std::make_unique<GridMapMapping>(
        resolution_, 
        width_, 
        height_, 
        occ_update_prob_, 
        free_update_prob_, 
        occ_threshold_, 
        free_threshold_
    );
    // Set up ROS publisher
    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    scan_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("mapping_scan_pc", 1);

    // Set up message filters for base pose and LiDAR scan, and register the callback
    scan_sub_.subscribe(nh_, "scan", 1);
    base_gt_odom_sub_.subscribe(nh_, "base_gt_odom", 1);
    sync_.registerCallback(boost::bind(&ViveMapperNode::syncCallback, this, _1, _2));
    ROS_INFO("ViveMapperNode initialized with stationary distance threshold: %f, stationary_angle_threshold: %f"
    ,stationary_d_threshold_,stationary_a_threshold_);

    laser_to_base_received_ = false;
    //set last base pose to a large negative value let the first map update can be performed
    last_x_ = -1000;
    last_y_ = -1000;
    last_yaw_ = -M_PI;
}

tf2::Transform ViveMapperNode::createtf2_from_XYyaw(double x, double y, double theta)
{
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(x, y, 0.0));
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    transform.setRotation(q);
    return transform;
}

void ViveMapperNode::tf2_listener_laser() 
{
    try{
        //"base_link is target,laser is source"
        laser_to_base_geoTf_ = tf_buffer_.lookupTransform(base_frame_id_, laser_frame_id_, ros::Time(0));
        tf2::fromMsg(laser_to_base_geoTf_.transform,laser_to_base_tf2_);
        ROS_INFO("Lidar to base x:%f, y:%f, z:%f",
        laser_to_base_tf2_.getOrigin().x()
        ,laser_to_base_tf2_.getOrigin().y()
        ,laser_to_base_tf2_.getOrigin().z());

        //get the lidar's YPR with respect to the base_link
        double yaw,pitch,roll;
        tf2::getEulerYPR(laser_to_base_tf2_.getRotation(),yaw,pitch,roll);
        ROS_INFO("Lidar to base yaw:%f, pitch:%f, roll:%f",yaw,pitch,roll);
        //have got the lidar to base tf
        laser_to_base_received_ = true;
    }
    catch (tf2::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
        // 处理异常
        return;
    }
}

void ViveMapperNode::get_laser_pose()
{   
    //check if we have the lidar to base tf
    if(!laser_to_base_received_)
    {
        tf2_listener_laser();
    }
    //get base to map transform
    base_to_map_tf2_ = createtf2_from_XYyaw(base_x_, base_y_, base_yaw_);
    //calculate laser to map transform
    laser_to_map_tf2_ = base_to_map_tf2_*laser_to_base_tf2_;
    laser_x_ = laser_to_map_tf2_.getOrigin().x();
    laser_y_ = laser_to_map_tf2_.getOrigin().y();
    laser_yaw_ = tf2::getYaw(laser_to_map_tf2_.getRotation());
    ROS_INFO("get LiDAR pose on map:  x: %f, y: %f, yaw: %f",laser_x_,laser_y_,laser_yaw_);

    //transform laser_to_map_tf2_ to eigen form
    laser_to_map_geoTf_ = tf2::toMsg(laser_to_map_tf2_);
    laser_to_map_eigen_ = tf2::transformToEigen(laser_to_map_geoTf_);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ViveMapperNode::create_scan_pc()
{   
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    ROS_INFO("start convert");
    float angle,x,y;
    pcl::PointXYZ point;
    ROS_INFO("original scan ranges's size: %zu",ranges_.size());
    // go through all the angles
    for (int i = 0; i < ranges_.size(); i++)
    {   
        // count the angle od the scan
        angle = laser_angle_min_ + i * laser_angle_increment_;
        if (ranges_[i] < range_max_ && ranges_[i] > range_min_)
        {
            // change into Cartesian coordinates
            x = ranges_[i] * cos(angle);
            y = ranges_[i] * sin(angle);

            // add the point into the points cloud
            point.x = x;
            point.y = y;
            point.z = 0.0;  
            pc->push_back(point);
            //ROS_INFO("scan to pc %d complete",i);
        }
        else
        {   
            // Convert angle to degrees for readability
            double angle_deg = angle * 180.0 / M_PI;
            ROS_WARN("Invalid range %f at angel %f(deg)", ranges_[i], angle_deg); 
        }
    }
    ROS_INFO("scan point cloud size: %zu",pc->size());
    ROS_INFO("scan to pc complete");
    return pc;
}



void ViveMapperNode::update_map(const ros::Time& scan_time)
{   
    //transform scan points from laser frame to map frame
    pcl::transformPointCloud(*scan_pc_,*scan_pc_on_map_,laser_to_map_eigen_.matrix());

    // Update the map by the scan points
    for (size_t i=0; i<scan_pc_on_map_->size(); i++) 
    {   
        //update map by the start point and end point
        grid_map_mapping_ptr->updateMap(laser_x_, laser_y_, scan_pc_on_map_->points[i].x, scan_pc_on_map_->points[i].y);
        //set z to 0 for visualization in RViz
        scan_pc_on_map_->points[i].z = 0.0;
    }  

    // Publish transformed point cloud for RViz visualization
    sensor_msgs::PointCloud2 scan_pc_on_map_msg;
    pcl::toROSMsg(*scan_pc_on_map_, scan_pc_on_map_msg);
    scan_pc_on_map_msg.header.frame_id = "map";  // Set the frame ID
    scan_pc_on_map_msg.header.stamp = scan_time;
    scan_pc_pub_.publish(scan_pc_on_map_msg);
    ROS_INFO("scan pc on map published");
}

bool ViveMapperNode::isStationary(double current_x, double current_y, double current_yaw) 
{
    // count the distance between the current pose and the previous pose
    double distance = sqrt(pow((current_x - last_x_), 2) + pow((current_y - last_y_), 2));
    //count the yaw angle change
    double yaw_difference = current_yaw - last_yaw_;
    //restrict the yaw difference in [-pi,pi] 
    if (yaw_difference > M_PI) 
    {
        yaw_difference -= 2.0 * M_PI;
    } 
    else if (yaw_difference < -M_PI) 
    {
        yaw_difference += 2.0 * M_PI;
    }

    bool stationary;
    // Check if the current pose is stationary based on the position threshold and the stationary duration
    if(distance < stationary_d_threshold_ && fabs(yaw_difference) < stationary_a_threshold_) 
    {
        stationary = true;
    } else 
    {
        stationary = false;
        ROS_INFO("distance is %f, yaw angle difference is %f ", distance, yaw_difference);
    }

    return stationary;
}

void ViveMapperNode::syncCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg, const nav_msgs::Odometry::ConstPtr& base_gt_odom_msg)
{   
    // Start timing
    ros::Time start_time = ros::Time::now();
    //get the parameters of the scan
    laser_angle_min_ = scan_msg->angle_min;
    laser_angle_max_ = scan_msg->angle_max;
    laser_angle_increment_ = scan_msg->angle_increment;
    laser_time_increment_ = scan_msg->time_increment;
    ROS_INFO("Received Laser Scan with angle min: %f, angle max: %f", scan_msg->angle_min, scan_msg->angle_max);
        
    ranges_ = scan_msg->ranges;
    //range_max_ = scan_msg->range_max;
    range_min_ = scan_msg->range_min;
    ROS_INFO("Laser Scan's range min: %f, range max: %f",range_min_ ,range_max_);

    // Get base pose
    base_x_ = base_gt_odom_msg->pose.pose.position.x;
    base_y_ = base_gt_odom_msg->pose.pose.position.y;
    base_yaw_ = tf2::getYaw(base_gt_odom_msg->pose.pose.orientation);
    ROS_INFO("base_link ground truth pose on map : x=%f, y=%f, yaw=%f",base_x_, base_y_, base_yaw_);

    // Check if the base is stationary
    if(isStationary(base_x_, base_y_, base_yaw_))
    {
        ROS_INFO("base_link is stationary\n");
    } else 
    {   
        last_x_ = base_x_;
        last_y_ = base_y_;
        last_yaw_ = base_yaw_;
        ROS_INFO("base_link is not stationary,so the mapper will perform map update");
        // Get laser pose
        get_laser_pose();
        // Convert laser scan to point cloud
        scan_pc_ = create_scan_pc();
        // transform points and update map
        update_map(scan_msg->header.stamp);
        ros::Time end_update_time = ros::Time::now();
        ros::Duration update_elapsed_time = end_update_time - start_time;
        ROS_INFO("Time taken for map update: %f seconds", update_elapsed_time.toSec());
        // Publish updated map
        map_pub_.publish(grid_map_mapping_ptr->toOccupancyGrid());
        ROS_INFO("Published updated map to 'map' topic.");
        // End timing
        ros::Time end_time = ros::Time::now();
        ros::Duration total_elapsed_time = end_time - start_time;

        ROS_INFO("Time taken for total map update and publication: %f seconds\n", total_elapsed_time.toSec());
    }


}

ViveMapperNode::~ViveMapperNode() {}

int main(int argc, char* argv[])
{
    ros::init(argc, argv,"Vive_mapper");
    ViveMapperNode vive_mapper_node;
    ros::spin();
    return 0;

}