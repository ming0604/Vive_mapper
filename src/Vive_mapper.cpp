#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2_ros/transform_listener.h>
#include "tf2/LinearMath/Transform.h"
#include <tf2/utils.h>

#include <nav_msgs/OccupancyGrid.h>
#include "Vive_mapper/grid_map_mapping.h"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, geometry_msgs::PoseStamped> MySyncPolicy;

class ViveMapperNode
{
    public:
        ViveMapperNode();
        virtual ~ViveMapperNode();
        void syncCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg, const geometry_msgs::PoseStamped::ConstPtr& base_gt_pose_msg);

        tf2::Transform createtf2_from_XYyaw(double x, double y, double theta);
        void tf2_listener_laser();
        void get_laser_pose(geometry_msgs::Pose base_gt_pose);

    private:
        // ROS NodeHandle, publishers, and subscribers
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        ros::Publisher map_pub_;
        message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub_;
        message_filters::Subscriber<geometry_msgs::PoseStamped> base_gt_pose_sub_;
        message_filters::Synchronizer<MySyncPolicy> sync_;
        
        // Map and pose variables
        GridMapMapping grid_map_mapping_;
        double base_x_, base_y_, base_yaw_;
        double laser_x_, laser_y_, laser_yaw_;

        //tf2,transform
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        tf2::Transform laser_to_base_tf2_;
        tf2::Transform base_to_map_tf2_;
        tf2::Transform laser_to_map_tf2_;
        geometry_msgs::TransformStamped laser_to_base_geoTf_;

        // Parameters
        double resolution_;
        int width_, height_;
        double occ_update_prob_, free_update_prob_;
        double occ_threshold_, free_threshold_;
        std::string base_frame_id_;
        std::string laser_frame_id_;

        //other signals
        bool laser_to_base_received;
};


ViveMapperNode::ViveMapperNode():nh_(), private_nh_("~"), tf_listener_(tf_buffer_), sync_(MySyncPolicy(10), scan_sub_, base_gt_pose_sub_)
{
    // Load parameters
    private_nh_.param("resolution", resolution_, 0.025);
    private_nh_.param("width", width_, 1000);
    private_nh_.param("height", height_, 1000);
    private_nh_.param("occ_update_prob", occ_update_prob_, 0.9);
    private_nh_.param("free_update_prob", free_update_prob_, 0.1);
    private_nh_.param("occ_threshold", occ_threshold_, 0.7);
    private_nh_.param("free_threshold", free_threshold_, 0.3);
    private_nh_.param("base_frame_id", base_frame_id_ ,"base_link");
    private_nh_.param("Lidar_frame_id", laser_frame_id_ ,"laser");

    // Initialize GridMapMapping with provided parameters
    grid_map_mapping_ = GridMapMapping(resolution_, width_, height_, occ_update_prob_, free_update_prob_, occ_threshold_, free_threshold_);

    // Set up ROS map publisher
    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);

    // Set up message filters for base pose and LiDAR scan, and register the callback
    scan_sub_.subscribe(nh_, "scan", 1);
    base_gt_pose_sub_.subscribe(nh_, "base_gt", 1);
    sync_.registerCallback(boost::bind(&ViveMapperNode::syncCallback, this, _1, _2));
    ROS_INFO("ViveMapperNode initialized");

    laser_to_base_received = false;
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

        //get the lidar's YPR with respect to the base_link
        double yaw,pitch,roll;
        tf2::getEulerYPR(laser_to_base_tf2_.getRotation(),yaw,pitch,roll);
        ROS_INFO("Lidar to base yaw:%f, pitch:%f, roll:%f",yaw,pitch,roll);
        //have got the lidar to base tf
        laser_to_base_received = true;
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
    if(!laser_to_base_received)
    {
        tf2_listener_laser();
    }
    //get base to map transform
    base_to_map_tf2_ = createtf2_from_XYyaw(base_x_, base_y_, base_yaw_);
    //calculate laser to map transform
    laser_to_map_tf2_ = base_to_map_tf2_*laser_to_base_tf2_;
    laser_x_ = laser_to_map_tf2.getOrigin().x();
    laser_y_ = laser_to_map_tf2.getOrigin().y();
    laser_yaw_ = tf2::getYaw(laser_to_map_tf2.getRotation());
    ROS_INFO("get LiDAR pose on map:  x: %f, y: %f, yaw: %f",laser_x_,laser_y_,laser_yaw_);
}
   

void ViveMapperNode::syncCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg, const geometry_msgs::PoseStamped::ConstPtr& base_gt_pose_msg)
{
    // Get base pose
    base_x_ = base_gt_pose_msg->pose.position.x;
    base_y_ = base_gt_pose_msg->pose.position.y;
    base_yaw_ = tf2::getYaw(base_gt_pose_msg->pose.orientation);
    ROS_INFO("base_link ground truth pose on map : x=%f, y=%f, yaw=%f",base_x_, base_y_, base_yaw_);
    // Get laser pose
    get_laser_pose();
    // Convert and process each laser scan point
    create_scan_pc()
    update_map();
    // Publish updated map
    map_pub_.publish(grid_map_mapping_.toOccupancyGrid());
    ROS_INFO("Published updated map to 'map' topic.");
}

ViveMapperNode::~ViveMapperNode() {}

int main(int argc, char* argv[])
{
    ros::init(argc, argv,"Vive_mapper");
    ViveMapperNode vive_mapper_node;
    ros::spin();
    return 0;

}