#include "segmentation_processor.h"
#include "point_processor.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <limits>
#include <algorithm>
#include <chrono>

SegmentationProcessor::SegmentationProcessor() : nh_("~") {
    initParams();
    segmentation_sub_ = nh_.subscribe(input_segment_topic_, 10, &SegmentationProcessor::segmentationCallback, this);
    boxes3d_pub_ = nh_.advertise<gb_visual_detection_3d_msgs::BoundingBoxes3d>(output_bbx3d_topic_, 100);
    markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/segmentation_processor/markers", 100);
    debug_pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(debug_pointcloud_topic_, 100);
    debug_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(debug_markers_topic_, 100);
    pointCloud_sub_.subscribe(nh_, pointcloud_topic_, 10);
    pointcloud_cache_.setCacheSize(100);
    pointcloud_cache_.connectInput(pointCloud_sub_);
    ROS_WARN("SegmentationProcessor initialized");
}

void SegmentationProcessor::initParams() {
    input_segment_topic_ = "/segmentation/objects_segment";
    output_bbx3d_topic_ = "/segmentation_processor/bounding_boxes_3d";
    pointcloud_topic_ = "/camera/depth_registered/points";
    debug_pointcloud_topic_ = "/segmentation_processor/debug_pointcloud";
    working_frame_ = "camera_link";  // Change to a valid frame if "map" is not available
    mininum_detection_threshold_ = 0.5f;
    minimum_probability_ = 0.3f;

    nh_.param("input_segment_topic", input_segment_topic_, input_segment_topic_);
    nh_.param("output_bbx3d_topic", output_bbx3d_topic_, output_bbx3d_topic_);
    nh_.param("point_cloud_topic", pointcloud_topic_, pointcloud_topic_);
    nh_.param("debug_pointcloud_topic", debug_pointcloud_topic_, debug_pointcloud_topic_);
    nh_.param("working_frame", working_frame_, working_frame_);
    nh_.param("mininum_detection_threshold", mininum_detection_threshold_, mininum_detection_threshold_);
    nh_.param("minimum_probability", minimum_probability_, minimum_probability_);
    ROS_WARN("Parameters initialized");
}

void SegmentationProcessor::segmentationCallback(const segmentation_msgs::ObjectsSegment::ConstPtr& msg) {
    ros::Time segment_stamp = msg->header.stamp;
    sensor_msgs::PointCloud2ConstPtr closest_pointcloud_msg = pointcloud_cache_.getElemBeforeTime(segment_stamp);

    if (!closest_pointcloud_msg) {
        ROS_WARN("No matching PointCloud2 message found for segment timestamp: %f", segment_stamp.toSec());
        return;
    }

    ROS_WARN("Found matching PointCloud2 message at: %f", closest_pointcloud_msg->header.stamp.toSec());

    sensor_msgs::PointCloud2 local_pointcloud;
    try {
        if (!tfListener_.waitForTransform(working_frame_, closest_pointcloud_msg->header.frame_id, segment_stamp, ros::Duration(1.0))) {
            ROS_WARN("No transform available from %s to %s", closest_pointcloud_msg->header.frame_id.c_str(), working_frame_.c_str());
            return;
        }

        pcl_ros::transformPointCloud(working_frame_, *closest_pointcloud_msg, local_pointcloud, tfListener_);
    } catch (tf::TransformException& ex) {
        ROS_ERROR_STREAM("Transform error: " << ex.what());
        return;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);
    try {
        pcl::fromROSMsg(local_pointcloud, *cloud_pcl);
    } catch (std::runtime_error& e) {
        ROS_ERROR("Error converting PointCloud2 to PCL: %s", e.what());
        return;
    }

    if (cloud_pcl->empty()) {
        ROS_ERROR("PointCloud is empty after conversion");
        return;
    }

    gb_visual_detection_3d_msgs::BoundingBoxes3d boxes3d_msg;
    boxes3d_msg.header.stamp = local_pointcloud.header.stamp;
    boxes3d_msg.header.frame_id = working_frame_;

    pcl::PointCloud<pcl::PointXYZRGB> debug_cloud;

    for (const auto& object_segment : msg->objects) {
        calculate_boxes(local_pointcloud, cloud_pcl, object_segment, &boxes3d_msg);

        // Add points from the segment to the debug point cloud
        for (size_t i = 0; i < object_segment.x_indices.size(); ++i) {
            int x = object_segment.x_indices[i];
            int y = object_segment.y_indices[i];

            if (x >= local_pointcloud.width || y >= local_pointcloud.height) {
                ROS_WARN("Point (%d, %d) is out of bounds for the point cloud", x, y);
                continue;
            }

            int pcl_index = (y * local_pointcloud.width) + x;
            const pcl::PointXYZRGB& point = cloud_pcl->at(pcl_index);
            if (!std::isnan(point.x)) {
                pcl::PointXYZRGB colored_point = point;
                // Color the point based on the object class
                colored_point.r = static_cast<uint8_t>((i + 1) * 50 % 255);
                colored_point.g = static_cast<uint8_t>((i + 1) * 100 % 255);
                colored_point.b = static_cast<uint8_t>((i + 1) * 150 % 255);
                debug_cloud.push_back(colored_point);
            }
        }
    }

    boxes3d_pub_.publish(boxes3d_msg);
    publish_markers(boxes3d_msg);
    publish_debug_pointcloud(debug_cloud);
    debug_markers_pub_.publish(center_markers_);
    center_markers_.markers.clear();
    ROS_WARN("3D bounding boxes and debug pointcloud published");
}

void SegmentationProcessor::publish_debug_pointcloud(const pcl::PointCloud<pcl::PointXYZRGB>& debug_cloud) {
    sensor_msgs::PointCloud2 debug_cloud_msg;
    pcl::toROSMsg(debug_cloud, debug_cloud_msg);
    debug_cloud_msg.header.frame_id = working_frame_;
    debug_cloud_msg.header.stamp = ros::Time::now();
    debug_pointcloud_pub_.publish(debug_cloud_msg);
}

void SegmentationProcessor::calculate_boxes(const sensor_msgs::PointCloud2& cloud_pc2,
                                            const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_pcl,
                                            const segmentation_msgs::ObjectSegment& object_segment,
                                            gb_visual_detection_3d_msgs::BoundingBoxes3d* boxes) {
    // Step 1: Create a point cloud for the object mask
    auto start = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    object_cloud->width = cloud_pc2.width;
    object_cloud->height = cloud_pc2.height;
    object_cloud->is_dense = false;
    object_cloud->points.resize(cloud_pc2.width * cloud_pc2.height);

    // Step 2: Populate object point cloud with points from the object mask
    for (size_t i = 0; i < object_segment.x_indices.size(); ++i) {
        int x = object_segment.x_indices[i];
        int y = object_segment.y_indices[i];

        if (x >= cloud_pc2.width || y >= cloud_pc2.height) {
            ROS_WARN("Point (%d, %d) is out of bounds for the point cloud", x, y);
            continue;
        }

        int pcl_index = (y * cloud_pc2.width) + x;
        const pcl::PointXYZRGB& point = cloud_pcl->at(pcl_index);

        if (std::isnan(point.x)) continue;

        object_cloud->at(pcl_index).x = point.x;
        object_cloud->at(pcl_index).y = point.y;
        object_cloud->at(pcl_index).z = point.z;
    }


    // Step 3-4: Segment the ground plane
    pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    int number_of_planes = 1;  // Segment one plane (ground)
    if (!segment_cloud(object_cloud, segmented_cloud, number_of_planes)) {
        ROS_WARN("Failed to segment point cloud");
        return;
    }

    // Step 5: Cluster the segmented cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr best_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    double cluster_tolerance = 0.07;  // Set clustering tolerance
    int min_cluster_size = 200;
    int max_cluster_size = 40000;
    if (!cluster_cloud(segmented_cloud, best_cluster, cluster_tolerance, min_cluster_size, max_cluster_size)) {
        ROS_WARN("Failed to cluster point cloud");
        return;
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end - start;
    std::cout << " ****########****** Clustering processing  took " << duration.count()
              << " milliseconds." << std::endl;

    // Step 6: Compute 3D bounding box from the best cluster
    float maxx, minx, maxy, miny, maxz, minz;
    maxx = maxy = maxz = -std::numeric_limits<float>::max();
    minx = miny = minz = std::numeric_limits<float>::max();

    for (const auto& point : best_cluster->points) {
        if (std::isnan(point.x)) continue;

        maxx = std::max(point.x, maxx);
        maxy = std::max(point.y, maxy);
        maxz = std::max(point.z, maxz);
        minx = std::min(point.x, minx);
        miny = std::min(point.y, miny);
        minz = std::min(point.z, minz);
    }

    // Step 7: Create and store the bounding box message
    gb_visual_detection_3d_msgs::BoundingBox3d bbx_msg;
    bbx_msg.Class = object_segment.class_name;
    bbx_msg.probability = object_segment.probability;
    bbx_msg.xmin = minx;
    bbx_msg.xmax = maxx;
    bbx_msg.ymin = miny;
    bbx_msg.ymax = maxy;
    bbx_msg.zmin = minz;
    bbx_msg.zmax = maxz;

    boxes->bounding_boxes.push_back(bbx_msg);
}


pcl::PointXYZRGB SegmentationProcessor::compute_center_point(const sensor_msgs::PointCloud2& cloud_pc2,
                                                             const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_pcl,
                                                             const segmentation_msgs::ObjectSegment& object_segment) {
    auto start = std::chrono::high_resolution_clock::now();

    if (object_segment.x_indices.empty() || object_segment.y_indices.empty()) {
        ROS_ERROR("Object segment has no valid indices");
        return pcl::PointXYZRGB();
    }

    int center_x = std::accumulate(object_segment.x_indices.begin(), object_segment.x_indices.end(), 0) / object_segment.x_indices.size();
    int center_y = std::accumulate(object_segment.y_indices.begin(), object_segment.y_indices.end(), 0) / object_segment.y_indices.size();
    std::vector<int> xs = object_segment.x_indices;
    std::vector<int> ys = object_segment.y_indices;
    std::sort(xs.begin(), xs.end());
    std::sort(ys.begin(), ys.end());

    int min_x = xs.front();
    int max_x = xs.back();
    int min_y = ys.front();
    int max_y = ys.back();

    int width = max_x - min_x;
    int height = max_y - min_y;

    // Convert num_samples to be in terms of height and width percentages
    int num_samples = 500; // Example number of samples
    float width_percentage = 0.12f;  // 55% of the bounding box width
    float height_percentage = 0.15f; // 35% of the bounding box height

    int width_step = static_cast<int>(width * width_percentage) / num_samples;
    int height_step = static_cast<int>(height * height_percentage) / num_samples;

    std::vector<pcl::PointXYZRGB> points;

    // Sampling in the region defined by the bounding box
    for (int i = -num_samples / 2; i <= num_samples / 2; ++i) {
        for (int j = -num_samples / 2; j <= num_samples / 2; ++j) {
            int new_x = center_x + i * width_step;
            int new_y = center_y + j * height_step;

            // Ensure the new points are within the image bounds
            if (new_x < 0 || new_x >= cloud_pc2.width || new_y < 0 || new_y >= cloud_pc2.height) {
                continue;
            }

            int pcl_index = (new_y * cloud_pc2.width) + new_x;
            pcl::PointXYZRGB point = cloud_pcl->at(pcl_index);

            // Check if the point is valid
            if (!std::isnan(point.x)) {
                points.push_back(point);
            }
        }
    }

    // If no valid points were found, return a default invalid point
    if (points.empty()) {
        return pcl::PointXYZRGB();
    }

    // Find the point with the minimum x value
    pcl::PointXYZRGB min_x_point = points[0];
    for (const auto& pt : points) {
        if (pt.x < min_x_point.x) {
            min_x_point = pt;
        }
    }
    // auto end = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double, std::milli> duration = end - start;
    // std::cout << "All the processing took " << duration.count()
    //           << " milliseconds." << std::endl;
    return min_x_point;
}
void SegmentationProcessor::publish_markers(const gb_visual_detection_3d_msgs::BoundingBoxes3d& boxes) {
    visualization_msgs::MarkerArray marker_array;

    for (size_t i = 0; i < boxes.bounding_boxes.size(); ++i) {
        const auto& box = boxes.bounding_boxes[i];

        visualization_msgs::Marker marker;
        marker.header.frame_id = boxes.header.frame_id;
        marker.header.stamp = boxes.header.stamp;
        marker.ns = "segmentation_3d";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = (box.xmax + box.xmin) / 2.0;
        marker.pose.position.y = (box.ymax + box.ymin) / 2.0;
        marker.pose.position.z = (box.zmax + box.zmin) / 2.0;
        marker.scale.x = (box.xmax - box.xmin);
        marker.scale.y = (box.ymax - box.ymin) ;
        marker.scale.z = box.zmax - box.zmin;

        marker.color.r = 1.0 - box.probability;
        marker.color.g = box.probability;
        marker.color.b = 0.0;
        marker.color.a = 0.6;

        marker_array.markers.push_back(marker);
    }

    markers_pub_.publish(marker_array);
}

void SegmentationProcessor::push_center_marker(const pcl::PointXYZRGB& center){

    static int i_ = 0;
     visualization_msgs::Marker marker;
     marker.header.frame_id = working_frame_;
     marker.header.stamp =  ros::Time::now();
     marker.ns = "segmentation_3d_CENTER";
     marker.id = i_++;
     marker.type = visualization_msgs::Marker::CUBE;
     marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = center.x;
        marker.pose.position.y = center.y;
        marker.pose.position.z = center.z;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        marker.color.r = 1.0;
        marker.color.g = 0.3;
        marker.color.b = 0.3;
        marker.color.a = 0.6;

        center_markers_.markers.push_back(marker);

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "segmentation_processor_node");
    SegmentationProcessor processor;
    ros::spin();
    return 0;
}