/* =========================================================== */
/* | Lidar Obstacle Detection on Real Streaming PCD          | */
/* |   Filename: processPointClouds.cpp                      | */
/* |   History:                                              | */
/* |    >> 15 July 2020: Optimized for Speed                 | */
/* |    >> 14 July 2020: Original Submission                 | */
/* |   Project Credits:                                      | */
/* |    >> Base Code Architecture by Aaron Brown             | */
/* |    >> PCL RANSAC, KD-Tree & Euclidean Clustering        | */
/* |       implemented with the help of class lecture notes  | */
/* =========================================================== */

#include "processPointClouds.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

// +-----------+
// | numPoints |
// +-----------+
template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(const typename pcl::PointCloud<PointT>::Ptr& cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

// +-------------+
// | FilterCloud |
// +-------------+
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr& cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    // Time segmentation process
    //auto startTime = std::chrono::steady_clock::now();
    
    // Perform Voxel Grid Downsampling
    pcl::VoxelGrid<PointT> vg; // create voxel grid
    
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>); // create new point cloud
    vg.setInputCloud(cloud); // give it the cloud
    vg.setLeafSize(filterRes,filterRes,filterRes); // define cell size, make sure there is only a single point for every cell
    vg.filter(*cloudFiltered); // results saved to cloudFiltered

    // Extract Point Cloud Region of Interest
    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>); // create new point cloud
    pcl::CropBox<PointT> region(true); // set region to true for points inside crop box
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    // Remove points scattered off the roof of our ego car
    std::vector<int> indices; // vector of ints for roof points
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices); // indices contain roof points

    // Redefine inliers as pointer to pass by reference
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for (int point : indices)
        inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloudRegion);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloudRegion); // remove roof points from cloud region

    //auto endTime = std::chrono::steady_clock::now();
    //auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    //std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;
}

// +-------------+
// | RansacPlane |
// +-------------+
template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlane(const typename pcl::PointCloud<PointT>::Ptr& cloud, int maxIterations, float distanceThreshold)
{
	std::unordered_set<int> inliersResult; // use this to hold best inliers
	srand(time(NULL));

	// Time segmentation process
    //auto startTime = std::chrono::steady_clock::now();

	// Shuffle/Randomize Point Cloud Indices
	//auto rng = std::default_random_engine {};
	//std::shuffle(cloud->points.begin(),cloud->points.end(),rng);

	while (maxIterations--) {

		// Create an unordered set to store inliers at the current iteration
		std::unordered_set<int> inliers;

		// Randomly pick points for plane
		while (inliers.size() < 3)
			inliers.insert(rand()%(cloud->points.size()));

		const int NUMPOINTS = inliers.size();
		float x[NUMPOINTS], y[NUMPOINTS], z[NUMPOINTS];
		int pointNum = 1;
		for (auto itr = inliers.begin(); itr != inliers.end(); itr++) {
			x[pointNum] = cloud->points[*itr].x;
			y[pointNum] = cloud->points[*itr].y;
			z[pointNum] = cloud->points[*itr].z;
			pointNum++;
		}

		// Find coefficients for the line formed by the 2 points selected above
		float A = (y[2]-y[1])*(z[3]-z[1]) - (z[2]-z[1])*(y[3]-y[1]);
		float B = (z[2]-z[1])*(x[3]-x[1]) - (x[2]-x[1])*(z[3]-z[1]);
		float C = (x[2]-x[1])*(y[3]-y[1]) - (y[2]-y[1])*(x[3]-x[1]);
		float D = -(A*x[1] + B*y[1] + C*z[1]);

		// run through all points in cloud and find inliers within distance tolerance
		for (int cloudIdx = 0; cloudIdx < cloud->points.size(); cloudIdx++)
		{
			// Compute the perpendicular distance from the current point to the fitted line
			float distance = fabs(A*cloud->points[cloudIdx].x + B*cloud->points[cloudIdx].y + C*cloud->points[cloudIdx].z + D)/sqrt(A*A+B*B+C*C);

			// if the distance is smaller than threshold
			if (distance <= distanceThreshold)
			    inliers.insert(cloudIdx); // then count it as an inlier for the current iteration/fitted line
		}

		// Keep track of the fitted model that has the greatest number of inliers
		if (inliers.size() > inliersResult.size())
		    inliersResult = inliers;
	}

	// End Timer
	//auto endTime = std::chrono::steady_clock::now();
	//auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	//std::cout << "RANSAC for Planes took " << elapsedTime.count() << " milliseconds" << std::endl;

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;
} // end ransac

// +--------------+
// | SegmentPlane |
// +--------------+
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(const typename pcl::PointCloud<PointT>::Ptr& cloud, const std::unordered_set<int>& inliers)
{
    // Time segmentation process
    //auto startTime = std::chrono::steady_clock::now();

    // Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());

    for(int index = 0; index < cloud->points.size(); index++) {
		if(inliers.count(index))
			planeCloud->points.push_back(cloud->points[index]);
		else
			obstCloud->points.push_back(cloud->points[index]);
	}

    // Return std::pair with the newly created obstacle and plane clouds
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);

    //auto endTime = std::chrono::steady_clock::now();
    //auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    //std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}

// +------------+
// | Clustering |
// +------------+
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(const typename pcl::PointCloud<PointT>::Ptr& cloud, float clusterTolerance, int minSize, int maxSize)
{
    // *********************************************************
    // Create a Kd-tree representation of the input point cloud 
    // *********************************************************

    // Time Kd-Tree Formation Process
    //auto startTime = std::chrono::steady_clock::now();

    // Reformat point cloud as a vector of point vectors for clustering
    std::vector<std::vector<float>> points;
        
    // Find the median x and y to start the tree (root), which may speed up the KdTree search process

    // Create a Kd-tree representation of the input point cloud       
    kdTree* kdtree = new kdTree; // Create the kdTree object for the search method of the extraction

    for (int idx = 0; idx < cloud->points.size(); idx++)
	{   
        std::vector<float> tmpPoints;   
        tmpPoints.push_back(cloud->points[idx].x);
        tmpPoints.push_back(cloud->points[idx].y);
        tmpPoints.push_back(cloud->points[idx].z);
        tmpPoints.push_back(cloud->points[idx].intensity);

        points.push_back(tmpPoints);
	}

    for (int pidx=0; pidx<points.size(); pidx++) // loop over all points in (obstacle) cloud
        kdtree->insert(points[pidx],pidx); // add node to tree

    //auto endTime = std::chrono::steady_clock::now();
    //auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    //std::cout << "Kd-tree took " << elapsedTime.count() << " milliseconds" << std::endl;

    // *********************************************************
    // Perform euclidean clustering to group detected obstacles
    // *********************************************************

    // Time Euclidean Clustering Process
    //auto startTime2 = std::chrono::steady_clock::now();

    std::vector<bool> processed(points.size(),false); // keep track of which points have been processed

    // Variable to Return: vector to store point cloud clusters
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

	int i = 0;
	while (i < points.size())
	{
		// if the point has already been processed, then increment counter and skip to the next point
		if (processed[i]) {
			i++;
			continue;
		}

		// if the point has not yet been processed ...
		std::vector<int> clusterVec; // create a new cluster, vector of ints
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

		// proximity function:
		clusterHelper(i, points, clusterVec, processed, kdtree, clusterTolerance); // i=pointID; clusterVec passed in as reference; tree=KDTree with search function
      
        for(int index : clusterVec)
        {
            cloudCluster->points.push_back(cloud->points[index]);

            // Kick out of loop if cluster gets larger than maxSize
            if (cloudCluster->points.size() > maxSize)
                break;
        }

        // Skip clusters that are smaller than minSize
        if (cloudCluster->points.size() < minSize)
            continue;

        // Skip clusters that are larger than maxSize
        if (cloudCluster->points.size() > maxSize)
            continue;

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        // stuff current point cloud cluster into vector point cloud clusters
        clusters.push_back(cloudCluster);

		i++;
	} 

    //auto endTime2 = std::chrono::steady_clock::now();
    //auto elapsedTime2 = std::chrono::duration_cast<std::chrono::milliseconds>(endTime2 - startTime2);
    //std::cout << "euclidean clustering took " << elapsedTime2.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters; // return list of indices for each cluster
}

// +---------------+
// | clusterHelper |
// +---------------+
template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int indice, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool>& processed, kdTree* kdtree, float distanceTol) 
{
	// Recursively form clusters of points, leveraging KDTree
	processed[indice] = true; // mark point as processed
	cluster.push_back(indice); // push that point back into cluster

	// Find which points are nearby relative to this indice
	std::vector<int> nearest = kdtree->search(points[indice],distanceTol);

	// Go through those nearby indices
	for (int id : nearest) { // grab nearest int IDs
		if (!processed[id]) { // if that point id has not been processed yet
			// then include it into clusterHelper
			clusterHelper(id, points, cluster, processed, kdtree, distanceTol);
		}
	}
}

// +--------------+
// | Bounding Box |
// +--------------+
template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(const typename pcl::PointCloud<PointT>::Ptr& cluster)
{
    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

// +---------+
// | savePcd |
// +---------+
template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(const typename pcl::PointCloud<PointT>::Ptr& cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}

// +---------+
// | loadPcd |
// +---------+
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    //std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}

// +-----------+
// | StreamPcd |
// +-----------+
template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{
    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}

// +-----------------+
// | SegmentPlanePCL |
// +-----------------+
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlanePCL(const typename pcl::PointCloud<PointT>::Ptr& cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    //auto startTime = std::chrono::steady_clock::now();

    // Find inliers for the cloud.
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Segment the largest planar component from the input cloud
    seg.setInputCloud(cloud); // perform segmentation on our cloud
    seg.segment (*inliers, *coefficients); // generate inliers and coefficients (latter can be used to render plane)
    // above: pass by reference, de-referencing inliers because it is declared above as a pointer
    // Inliers will point to the list of indices that belong to the fitted plane that we found by doing RANSAC
    // Use inliers to separate the point cloud into two pieces

    // If we didn't find any model that can fit this data ...
    if(inliers->indices.size() == 0)
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;

    // Use inliers to create the plane point cloud and the obstacle point cloud
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateCloudsPCL(inliers,cloud);

    //auto endTime = std::chrono::steady_clock::now();
    //auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    //std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}

// +-------------------+
// | SeparateCloudsPCL |
// +-------------------+
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateCloudsPCL(pcl::PointIndices::Ptr& inliers, const typename pcl::PointCloud<PointT>::Ptr& cloud) 
{
    // Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());

    // Generate Plane Cloud
    // Add inliers to the plane cloud by looping over the inlier indices and pushing the corresponding inlier point into the plane cloud's point vector
    for(int index : inliers->indices) // for all the inliers
    {
        // grab member points from the reference cloud (the PointT that lies at that index)
        planeCloud->points.push_back(cloud->points[index]);
    }

    // Generate Obstacle Cloud
    // Use an extract object to subtract the plane cloud from the input cloud, leaving us with the obstacle cloud
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud); // send reference cloud
    extract.setIndices (inliers);  // send inliers
    extract.setNegative (true);    // set inliers to negative
    extract.filter (*obstCloud);   // de-reference obstacle cloud pointer when passing into filter, remove inliers from reference cloud

    // Return std::pair with the newly created obstacle and plane clouds
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}

// +---------------+
// | ClusteringPCL |
// +---------------+
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringPCL(const typename pcl::PointCloud<PointT>::Ptr& cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    //auto startTime = std::chrono::steady_clock::now();

    // Create a vector to store point clouds
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Perform euclidean clustering to group detected obstacles
    // Create the KDTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    // Populate output cluster vector based on clusterIndices
    for (pcl::PointIndices getIndices: clusterIndices) {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for(int index : getIndices.indices) {
            cloudCluster->points.push_back (cloud->points[index]);
        }
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }

    //auto endTime = std::chrono::steady_clock::now();
    //auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    //std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}