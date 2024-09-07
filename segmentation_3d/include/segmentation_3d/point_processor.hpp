#ifndef _POINT_PROCESSOR_HPP_
#define _POINT_PROCESSOR_HPP_
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <Eigen/Dense>
#include <map>
#include <tuple>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>


bool filter_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered,
                        double leaf_size);

bool segment_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_segmented,
                   const int number_of_planes);

bool cluster_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr best_cluster,
                   double cluster_tolerance,
                   int min_cluster_size,
                   int max_cluster_size);


#endif // _POINT_PROCESSOR_HPP_

