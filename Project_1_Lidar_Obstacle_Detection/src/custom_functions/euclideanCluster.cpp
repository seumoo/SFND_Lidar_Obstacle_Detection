/* 
Created by Sue Nopachinda
*/

#include "kdtree.h"


void clusterHelper(int idx, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol) {
	// if the point was not processed: (1) mark it as processed. (2) add the point to the cluster, (3) iterate the nearby points
	processed[idx] = true;
	cluster.push_back(idx);
	std::vector<int> nearest = tree->search(points[idx], distanceTol);

	for (int id : nearest) {
		if (!processed[id]) clusterHelper(id, points, cluster, processed, tree, distanceTol);
	}
}

/* 
    EuclideanClustering 
    Uses Euclidean Clustering to determine groups of nearest points and forms a cluster to be defined as a singlton object.
    Code based in the course lesson and reuses code from the function 'euclideanCluster()' in 'src/quiz/cluster/cluster.cpp'.
*/
std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{
	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(points.size(), false);		// vector which stores a bool if point has been processed or not

	int i = 0;
	while(i < points.size()) 
	{
		if (processed[i]) 	// if the point was processed already, skip and continue to the next point
		{			
			i++;
			continue;
		}

		// if the point was not processed: (1) mark it as processed. (2) add the point to the cluster, (3) iterate the nearby points
		std::vector<int> cluster;
		clusterHelper(i, points, cluster, processed, tree, distanceTol);
		clusters.push_back(cluster);
		i++;
	}

	return clusters;
}