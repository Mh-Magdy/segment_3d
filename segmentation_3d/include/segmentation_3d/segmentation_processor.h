#ifndef SEGMENTATION_PROCESSOR_H
#define SEGMENTATION_PROCESSOR_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <segmentation_msgs/ObjectsSegment.h>
#include <segmentation_msgs/ObjectSegment.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

#include <gb_visual_detection_3d_msgs/BoundingBoxes3d.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_ros/point_cloud.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>

#include <string>

class SegmentationProcessor {
public:
    SegmentationProcessor();
    void segmentationCallback(const segmentation_msgs::ObjectsSegment::ConstPtr& msg);
    void initParams();
    void publish_markers(const gb_visual_detection_3d_msgs::BoundingBoxes3d& boxes);
    void calculate_boxes(const sensor_msgs::PointCloud2& cloud_pc2,
                         const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_pcl,
                         const segmentation_msgs::ObjectSegment& object_segment,
                         gb_visual_detection_3d_msgs::BoundingBoxes3d* boxes);
    void publish_debug_pointcloud(const pcl::PointCloud<pcl::PointXYZRGB>& debug_cloud);
  
    pcl::PointXYZRGB compute_center_point(const sensor_msgs::PointCloud2& cloud_pc2,
                                          const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_pcl,
                                          const segmentation_msgs::ObjectSegment& object_segment);
    void push_center_marker(const pcl::PointXYZRGB& center);

private:
    ros::NodeHandle nh_;
    ros::Subscriber segmentation_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointCloud_sub_;
    message_filters::Cache<sensor_msgs::PointCloud2> pointcloud_cache_;

    ros::Publisher boxes3d_pub_, markers_pub_, debug_pointcloud_pub_, debug_markers_pub_;
    tf::TransformListener tfListener_;

    std::string input_segment_topic_;
    std::string output_bbx3d_topic_;
    std::string pointcloud_topic_;
    std::string debug_pointcloud_topic_;
    std::string debug_markers_topic_;
    std::string working_frame_;
    std::vector<std::string> interested_classes_;
    float mininum_detection_threshold_, minimum_probability_;
    visualization_msgs::MarkerArray center_markers_;
};

#endif // SEGMENTATION_PROCESSOR_H
