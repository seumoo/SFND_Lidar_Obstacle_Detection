/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// Add inliers
	float scatter = 0.6;
	for (int i = -5; i < 5; i++)
	{
		double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		pcl::PointXYZ point;
		point.x = i + scatter * rx;
		point.y = i + scatter * ry;
		point.z = 0;

		cloud->points.push_back(point);
	}
	// Add outliers
	int numOutliers = 10;
	while (numOutliers--)
	{
		double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		pcl::PointXYZ point;
		point.x = 5 * rx;
		point.y = 5 * ry;
		point.z = 0;

		cloud->points.push_back(point);
	}
	cloud->width = cloud->points.size();
	cloud->height = 1;

	return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
	viewer->addCoordinateSystem(1.0);
	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// TODO: Fill in this function

	// For max iterations
	while (maxIterations--)
	{
		// randomly pick 2 points to create a line
		// Randomly sample subset and fit line

		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier

		// Return indicies of inliers from fitted line with most inliers

		std::unordered_set<int> inliers;
		while (inliers.size() < 2)
			inliers.insert(rand() % (cloud->points.size())); // keep selecting points until there are 2. Use % to ensure the point fits in the cloud space

		float x1, y1, x2, y2;
		auto itr = inliers.begin(); // get the first point
		x1 = cloud->points[*itr].x; // dereference the iterator using *itr
		y1 = cloud->points[*itr].y;
		itr++;						// move to the next point
		x2 = cloud->points[*itr].x; // dereference the iterator using *itr
		y2 = cloud->points[*itr].y;

		float a = y1 - y2;
		float b = x2 - x1;
		float c = (x1 * y2 - x2 * y1);

		for (int index = 0; index < cloud->points.size(); index++)
		{
			// check to see if the 2 points chosen are the current point we are proccessing. if it is, skip it
			if (inliers.count(index) > 0)
			{
				continue;
			}

			pcl::PointXYZ point = cloud->points[index]; // no z point to simply it to be a 2d model
			float x3 = point.x;
			float y3 = point.y;

			float distance = fabs(a * x3 + b * y3 + c) / sqrt(a * a + b * b); // fabs = float absolute value
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
	std::cout << "Ransac took " << elapsedTime.count() << " milliseconds." << std::endl;
	return inliersResult;
}

// use 3 points to intersect a plane
std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
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

			pcl::PointXYZ point = cloud->points[index];
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

int main()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac3D(cloud, 50, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for (int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if (inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	// Render 2D point cloud with inliers and outliers
	if (inliers.size())
	{
		renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
		renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
	}
	else
	{
		renderPointCloud(viewer, cloud, "data");
	}

	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}
}
