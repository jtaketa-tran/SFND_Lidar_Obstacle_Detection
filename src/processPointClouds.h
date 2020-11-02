/* =========================================================== */
/* | Lidar Obstacle Detection on Real Streaming PCD          | */
/* |   Filename: processPointClouds.h                        | */
/* |   History:                                              | */
/* |    >> 15 July 2020: Optimized for Speed                 | */
/* |    >> 14 July 2020: Original Submission                 | */
/* |   Project Credits:                                      | */
/* |    >> Base Code Architecture by Aaron Brown             | */
/* |    >> PCL RANSAC, KD-Tree & Euclidean Clustering        | */
/* |       implemented with the help of class lecture notes  | */
/* =========================================================== */

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

// PCL lib Functions for processing point clouds 
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"
#include "kdTree.h"
#include <unordered_set>

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(const typename pcl::PointCloud<PointT>::Ptr& cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr& cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::unordered_set<int> RansacPlane(const typename pcl::PointCloud<PointT>::Ptr& cloud, int maxIterations, float distanceThreshold);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateCloudsPCL(pcl::PointIndices::Ptr& inliers, const typename pcl::PointCloud<PointT>::Ptr& cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(const typename pcl::PointCloud<PointT>::Ptr& cloud, const std::unordered_set<int>& inliers);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlanePCL(const typename pcl::PointCloud<PointT>::Ptr& cloud, int maxIterations, float distanceThreshold);
    
    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(const typename pcl::PointCloud<PointT>::Ptr& cloud, float clusterTolerance, int minSize, int maxSize);
    std::vector<typename pcl::PointCloud<PointT>::Ptr> ClusteringPCL(const typename pcl::PointCloud<PointT>::Ptr& cloud, float clusterTolerance, int minSize, int maxSize);

    void clusterHelper(int indice, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool>& processed, kdTree* kdtree, float clusterTolerance);

    Box BoundingBox(const typename pcl::PointCloud<PointT>::Ptr& cluster);

    void savePcd(const typename pcl::PointCloud<PointT>::Ptr& cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
};
#endif /* PROCESSPOINTCLOUDS_H_ */