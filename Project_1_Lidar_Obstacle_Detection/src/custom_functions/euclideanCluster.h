
/* Created by Sue Nopachinda
 */

#ifndef EUCLIDEANCLUSTER_H
#define EUCLIDEANCLUSTER_H
#include "kdtree.h"

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol);

void clusterHelper(int idx, const std::vector<std::vector<float>> points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol);

#endif /* EUCLIDEANCLUSTER_H */
