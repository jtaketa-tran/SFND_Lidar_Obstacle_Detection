/* =========================================================== */
/* | Lidar Obstacle Detection on Real Streaming PCD          | */
/* |   Filename: environment.cpp                             | */
/* |   History:                                              | */
/* |    >> 15 July 2020: Optimized for Speed                 | */
/* |    >> 14 July 2020: Original Submission                 | */
/* |   Project Credits:                                      | */
/* |    >> Base Code Architecture by Aaron Brown             | */
/* |    >> PCL RANSAC, KD-Tree & Euclidean Clustering        | */
/* |       implemented with the help of class lecture notes  | */
/* =========================================================== */

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
#include "processPointClouds.cpp" // using templates for processPointClouds so also include .cpp to help linker

// +-----------+
// | cityBlock |
// +-----------+
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> pointProcessorI, pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    renderPointCloud(viewer,inputCloud,"inputCloud");
    
    float voxelGridSize = 0.13; // 0.1
    Eigen::Vector4f minPoint = Eigen::Vector4f(-10,-5,-2,1); 
    Eigen::Vector4f maxPoint = Eigen::Vector4f(30,7,4,1); // (30,8,3,1)
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI.FilterCloud(inputCloud,voxelGridSize,minPoint,maxPoint);
    //renderPointCloud(viewer,filterCloud,"filterCloud");
    
    // Find Points Belonging to the Plane
    int   maxIterations     = 25;
    float distanceThreshold = 0.2;
    //std::unordered_set<int> inliers = pointProcessorI.RansacPlane(filterCloud,maxIterations,distanceThreshold);
    // Segment Point Cloud into Plane and Obstacles
    //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI.SegmentPlane(filterCloud,inliers);
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI.SegmentPlanePCL(filterCloud,maxIterations,distanceThreshold);
    //renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));   // red
    //renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0)); // green

    // Separate obstacle point cloud into clusters
    float clusterTol     = 0.3;
    int   minClusterSize = 20; //30;
    int   maxClusterSize = 1400; //1850;
    //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI.Clustering(segmentCloud.first,clusterTol,minClusterSize,maxClusterSize);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI.ClusteringPCL(segmentCloud.first,clusterTol,minClusterSize,maxClusterSize);

    // Iterate through vector of point clouds 
    int clusterID = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,0.6,0.3), Color(0,0,1), Color(1,1,0), Color(0.94,0.97,1), Color(1,0.27,0), Color(0.98,0.5,0.45), Color(1,0.65,0)}; // red, green, blue, yellow, light blue
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {    
        // Render Clusters
        //std::cout << "cluster size ";
        //pointProcessorI.numPoints(cluster);
        //temp remove: renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterID),colors[clusterID%colors.size()]);
               
        // Render Box
        Box box = pointProcessorI.BoundingBox(cluster);
        renderBox(viewer,box,clusterID,Color(0,0,1)); //(0.54,0.17,0.89));

        ++clusterID;
    }
    //renderPointCloud(viewer,segmentCloud.second,"planeCloud");
}

// +-------------+
// | initHighway |
// +-------------+
std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

/*
// +---------------+
// | simpleHighway |
// +---------------+
void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    bool renderScene = false; //true;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // Create lidar sensor
    Lidar* lidar = new Lidar(cars,0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    //renderRays(viewer,lidar->position,inputCloud);
    //renderPointCloud(viewer,inputCloud,"inputCloud");

    // Call point processor function on the input cloud and render the two segmented point clouds in different colors
    ProcessPointClouds<pcl::PointXYZ> pointProcessor; // instantiate on the stack
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlanePCL(inputCloud,100,0.2);

    // Render point cloud has color options {Red, Green, Blue}. Default color setting is white.
    //renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));   // red
    //renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0)); // green

    // Separate obstacle point cloud into clusters
    float clusterTol = 1.0;
    int minClusterSize = 3;
    int maxClusterSize = 30;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.ClusteringPCL(segmentCloud.first,clusterTol,minClusterSize,maxClusterSize);

    // iterate through vector of point clouds 
    int clusterID = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)}; // red, yellow, blue
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {    
        // Render clusters with cycling over colors
        std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterID),colors[clusterID%colors.size()]);
               
        // Render bounding box for obstacles (cars)
        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer,box,clusterID);

        ++clusterID;
    }
    renderPointCloud(viewer,segmentCloud.second,"planeCloud");
}
*/

// +------------+
// | initCamera |
// +------------+
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    viewer->setBackgroundColor (0, 0, 0);    
    viewer->initCameraParameters(); // set camera position and angle
    int distance = 16; // distance away in meters
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);  
}

// +------+
// | main |
// +------+
int main (int argc, char** argv)
{
    //std::cout << "starting environment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS; //XY;
    initCamera(setAngle, viewer);

    // Create a new point processor
    ProcessPointClouds<pcl::PointXYZI> pointProcessorI; // instantiate on the stack

    // Stream Point Cloud Data from File
    std::vector<boost::filesystem::path> stream = pointProcessorI.streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();

    // Create placeholder for point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    //renderPointCloud(viewer,inputCloud,"inputCloud");

    // PCL Viewer Update Loop    
    while (!viewer->wasStopped ()) // While the pcl viewer hasn't stopped
    {
        // Clear any previous rendered point clouds or shapes
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load new frame of point cloud data, de-referencing stream iterator pathname and convert to string
        inputCloudI = pointProcessorI.loadPcd((*streamIterator).string());

        // Run Obstacle Detection
        cityBlock(viewer,pointProcessorI,inputCloudI);

        // Update iterator over the stream vector
        streamIterator++;
        // If the iterator hits the end of the vector, then set it back to the beginning
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        // Controls the frame rate, by default it waits 1 time step
        viewer->spinOnce ();
    } 
}
