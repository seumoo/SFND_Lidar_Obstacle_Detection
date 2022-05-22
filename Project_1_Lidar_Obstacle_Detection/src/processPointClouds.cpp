// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "custom_functions/euclideanCluster.h"
#include <unordered_set>

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

/* 
    FilterCloud
    Applies Voxel grid filtering to a point cloud to reduce the number of points by cropping a region to process.
    This makes clustering much faster as there are fewer points to process verses an entire scene. 
*/
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Voxel grid point reduction
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered{new pcl::PointCloud<PointT>};
    pcl::VoxelGrid<PointT> voxelGridFilter;
    voxelGridFilter.setInputCloud(cloud);
    voxelGridFilter.setLeafSize(filterRes, filterRes, filterRes);
    voxelGridFilter.filter(*cloudFiltered);

    // use crop box to select the region of interest and keep these points for processing
    typename pcl::PointCloud<PointT>::Ptr cloudRegion{new pcl::PointCloud<PointT>}; // the resultant cloud data after applying a crop box (region of interest)
    pcl::CropBox<PointT> cropBoxFilter(true);   // True = remove these indices from the resultant cloud data
    cropBoxFilter.setMin(minPoint);
    cropBoxFilter.setMax(maxPoint);
    cropBoxFilter.setInputCloud(cloudFiltered);
    cropBoxFilter.filter(*cloudRegion);

    // Get the indices of rooftop points
    std::vector<int> roofIndices;   // holds the ego car roof points to be removed later
    pcl::CropBox<PointT> roofFilter(true);    // CropBox (bool extract_removed_indices=false). True = remove these indices from the resultant cloud data
    const Eigen::Vector4f minRoof(-1.5, -1.7, -1, 1);   // the renderBox() function from enviornment.cpp was used to estimate the roof bounding box
    const Eigen::Vector4f maxRoof(2.6, 1.7, 0.4, 1);
    roofFilter.setMin(minRoof);
    roofFilter.setMax(maxRoof);
    roofFilter.setInputCloud(cloudRegion);
    roofFilter.filter(roofIndices);

    // add the indices to PointIndices pointer reference to be used when removing roof points out of the PCD
    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};  
    for (int point: roofIndices)
        inliers->indices.push_back(point);  // indicies object built-in into inliers PointIndicies object

    // seperate the roof indices from the point cloud
    pcl::ExtractIndices<PointT> extract;
    extract.setIndices(inliers);
    extract.setNegative(true);  // remove the indices points
    extract.setInputCloud(cloudRegion);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "downsampled original " << cloud->points.size() << " points to " << cloudRegion->points.size() << std::endl;
    std::cout << "filtering cloud took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;
}

/* 
    SeparateClouds 
    Seperates inlier points (road points) from the overall cloud data.
    Returns the road cloud and obstacle cloud.
*/
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());

    // iterate through the inliers (road plane) and add the associate cloud points to the planeCloud object. 
    // the planeCloud object will be used to seperate the overall point cloud with road and wanted obstacles (cars, trees)
    // use -> because inliers is a pointer object
    for (int inlierIdx : inliers->indices)
    {
        planeCloud->points.push_back(cloud->points[inlierIdx]);
    }

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);   // the entire point cloud data
    extract.setIndices(inliers);    // used to seperate plane - road points
    extract.setNegative(true);  // [in]	negative	false = normal filter behavior (default), true = inverted behavior.
    extract.filter(*obstacleCloud);   // seperates the cloud data into obstacles and the road plane. All the points that are not inliers are kept here

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, planeCloud);
    return segResult;
}

/* 
    SegmentPlane 
    Returns pair of segmented point clouds: obstacle points / road points using PCL built-in function SAC Segmentation.
    Inliers are the road points.
*/
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients()); // define coefficients for the model - used to define plane and can be rendered in PCL viewer
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());    // used to seperate point cloud into 2 pieces
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;

    // Optional
    seg.setOptimizeCoefficients(true);      // try to get best model
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);  
    seg.setMethodType(pcl::SAC_RANSAC); // random sample concensus
    seg.setMaxIterations(maxIterations);    
    seg.setDistanceThreshold(distanceThreshold);

    // segment the largest planar component from the input cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);    // coefficients can be used to render plane. inliars used to seperate plane (road points)

    if (inliers->indices.size() == 0)   // didn't find any model that can fit this data
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
 
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}

// seperate the cloud between road and objects using Ransac method.
// code is re-used from quiz/RansacQuiz.cpp
template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac3DHelper(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// TODO: Fill in this function

	// For max iterations
	while (maxIterations--)
	{
		// randomly pick 3 points to create a plane

		std::unordered_set<int> inliers;
		while (inliers.size() < 3)
			inliers.insert(rand() % (cloud->points.size())); // keep selecting points until there are 3. Use % to ensure the point fits in the cloud space

		float x1, y1, z1, x2, y2, z2, x3, y3, z3;
		auto itr = inliers.begin(); // get the first point
		x1 = cloud->points[*itr].x; // dereference the iterator using *itr
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;						// move to the next point
		x2 = cloud->points[*itr].x; // dereference the iterator using *itr
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;						// move to the next point
		x3 = cloud->points[*itr].x; // dereference the iterator using *itr
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		// Ax + By + Cz + D = 0
		float a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
		float b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
		float c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
		float d = -1 * (a * x1 + b * y1 + c * z1);

		for (int index = 0; index < cloud->points.size(); index++)
		{
			// check to see if the 3 points chosen are the current point we are proccessing. if it is, skip it
			if (inliers.count(index) > 0)
			{
				continue;
			}

			PointT point = cloud->points[index];
			float x3 = point.x;
			float y3 = point.y;
			float z3 = point.z;

			float distance = fabs(a * x3 + b * y3 + c * z3 + d) / sqrt(a * a + b * b + c * c); // fabs = float absolute value
			if (distance <= distanceTol)
			{
				// if the distance is less than or equal to the tolerance, it's close to the line and is added as an inlier point
				inliers.insert(index);
			}
		}

		if (inliers.size() > inliersResult.size())
			inliersResult = inliers; // if there are more inliers in this iteration, it's the best so far
	}

	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "Ransac3D took " << elapsedTime.count() << " milliseconds." << std::endl;
	return inliersResult;
}

/* 
    SegmentPlaneRansac 
    RANSAC implementation for spliting the point cloud between road plane and object planes.
*/
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneRansac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    std::unordered_set<int> cloudInliers = Ransac3DHelper(cloud, maxIterations, distanceThreshold); // provides a set of road points. use this to segment the cloud between road plane and objects
    pcl::PointIndices::Ptr inlierIndicies{new pcl::PointIndices};

    // add the inliers from the input cloud into PointIndicies object. This will be used as an input for the SeparateClouds function
    for(const int index : cloudInliers) {
        inlierIndicies->indices.push_back(index);   
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segmentedCloud = SeparateClouds(inlierIndicies,cloud);
    return segmentedCloud;
}


/* 
    Clustering 
    PCL built-in segmentation. Uses Euclidean Clustering to determine groups of nearest points and forms a cluster
    to be defined as a singlton object.
*/
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance); // 2cm
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (pcl::PointIndices getIndices : cluster_indices) {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);  // holds obstacles in the point cloud
        for (int index : getIndices.indices) {
            cloud_cluster->points.push_back(cloud->points[index]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

/* 
    EuclideanClustering 
    Custom built function to cluster points via Euclidean Clustering algorithm.
*/
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::EuclideanClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters; // stores the found clusters into a point cloud object
    std::vector<std::vector<float>> cloudPoints;    // stores the cloud points as input for the euclideanCluster() function

	KdTree* tree = new KdTree;
    for (int i = 0; i < cloud->points.size(); i++) {
        PointT point = cloud->points[i];
        std::vector<float> pointVector = {point.x, point.y, point.z};
        tree->insert(pointVector, i);
        cloudPoints.push_back(pointVector);
    }
        
    std::vector<std::vector<int>> euclideanClusters = euclideanCluster(cloudPoints, tree, clusterTolerance);
    // add the clusters to the point cloud
    for(std::vector<int> cluster : euclideanClusters)
  	{
        // check if the points within the cluster statisfy the min and max size specified from the function arguments
        if (cluster.size() >= minSize && cluster.size() < maxSize)
        {
            typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>); // holds obstacles in the point cloud
            for (int index: cluster)
            {
                cloud_cluster->points.push_back(cloud->points[index]);
            }
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            clusters.push_back(cloud_cluster);
        }
  	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Euclidean clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}




/* 
    BoundingBox 
    Creates a Box object which encompasses a defined cluster.
*/
template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
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


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}

