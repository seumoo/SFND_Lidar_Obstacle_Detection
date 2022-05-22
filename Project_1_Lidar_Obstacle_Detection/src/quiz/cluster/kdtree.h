/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	// ** is a memory address of the node. uint = unsigned integer can never be negative
	void insertHelper(Node *&node, uint depth, std::vector<float> point, int id) {
		// tree is empty, create a new node as the root
		if (node == NULL) {
			node = new Node(point, id);
		}
		else {
			// if the depth is even, split the x region. if the depth is odd, split the y region.
			uint dimension = depth % 2;	// even = 0; odd == 1
			float currentValue = point[dimension];	// x or y value depending on the dimension
			float compareValue = node->point[dimension];
			if (currentValue < compareValue) {
				// if less than, go left
				insertHelper(node->left, depth+1, point, id);
			}
			else {
				// if more than, go right
				insertHelper(node->right, depth+1, point, id);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(root, 0, point, id);	// &root =  memory address of the root node

	}

	// pass the ids vector by reference since we are modifying it for every iteration
	void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids) {
		if (node != NULL) {
			float minX = target[0] - distanceTol;
			float maxX = target[0] + distanceTol;
			float minY = target[1] - distanceTol;
			float maxY = target[1] + distanceTol;

			// check if the point is in the box boundaries
			if (node->point[0] >= minX && node->point[0] <= maxX && node->point[1] >= minY && node->point[1] <= maxY) {
				float distance = sqrt( ((node->point[0] - target[0]) * (node->point[0] - target[0])) + ((node->point[1] - target[1]) * (node->point[1] - target[1])) );
				if (distance <= distanceTol) 
				{
					cout << "found point: " << node->id << "; distance diff: " << distance << "\n";
					ids.push_back(node->id);
				}
			}
			// check if the box crosses over the node division region
			int boundary = depth % 2;	// x or y point index
			if( (target[boundary] - distanceTol) < node->point[boundary] ) searchHelper(target, node->left, depth + 1, distanceTol, ids);
			if( (target[boundary] + distanceTol) > node->point[boundary] ) searchHelper(target, node->right, depth + 1, distanceTol, ids);			
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};




