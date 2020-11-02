/* =========================================================== */
/* | Lidar Obstacle Detection on Real Streaming PCD          | */
/* |   Filename: kdTree.h                                    | */
/* |   History:                                              | */
/* |    >> 15 July 2020: Optimized for Speed                 | */
/* |    >> 14 July 2020: Original Submission                 | */
/* |   Project Credits:                                      | */
/* |    >> Base Code Architecture by Aaron Brown             | */
/* |    >> PCL RANSAC, KD-Tree & Euclidean Clustering        | */
/* |       implemented with the help of class lecture notes  | */
/* =========================================================== */

#ifndef KDTREE_H
#define KDTREE_H

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float>& arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct kdTree
{
	Node* root;

	kdTree()
	: root(NULL)
	{}

/*
	// Double Pointer Implementation:
	// Pass in a pointer to the node, starting at root. Node* is the root. 
	// Node** is the memory address that is pointing at the node were are currently on in the tree (double pointer).
	// Replace a node by de-referencing the double pointer and assigning it to the newly created Node
	void insertHelper(Node **node, uint depth, std::vector<float> point, int id) {
		// This recursive helper function traverses a tree until the Node it arrives at is NULL, 
		// at which point a new Node is created in place of that NULL Node

		if (*node == NULL) // de-reference node pointer to see what it's value is
			*node = new Node(point, id); // de-reference node and set it; tell root node that this is the new data we should be pointing at
		else {
			// Traverse and split the tree regions across x and y dimensions to find the NULL node to replace
			uint currentdepth = depth % 3; // mod 3 because this is a 3D case: is the depth even or odd?
			// if currentdepth is even/0, we are comparing the x-values; if currentdepth is odd/1, we are comparing the y-values
			if(point[currentdepth] < ((*node)->point[currentdepth]))
				insertHelper(&((*node)->left), depth+1, point, id);
			else // new point (x or y value) is greater than or equal to the current *node value
				insertHelper(&((*node)->right), depth+1, point, id);
		}
		// the recursion terminates when we hit a NULL node
	}
*/
	// Pointer Reference Implementation:
	void insertHelper(Node *&node, uint depth, std::vector<float>& point, int id) {
		// This recursive helper function traverses a tree until the Node it arrives at is NULL, 
		// at which point a new Node is created in place of that NULL Node

		if (node == NULL)
			node = new Node(point, id);
		else {
			// Calculate current depth
			uint cd = depth % 3;
			if(point[cd] < node->point[cd])
				insertHelper(node->left, depth+1, point, id);
			else
				insertHelper(node->right, depth+1, point, id);
		}
	}

	void searchHelper(std::vector<float>& target, Node*node, int depth, float distanceTol, std::vector<int>& ids) {
		if (node != NULL) {
			// see if node is within the target bounding box
			if ((node->point[0]>=(target[0]-distanceTol)&&node->point[0]<=(target[0]+distanceTol)) 
			&& (node->point[1]>=(target[1]-distanceTol)&&node->point[1]<=(target[1]+distanceTol))
			&& (node->point[2]>=(target[2]-distanceTol)&&node->point[2]<=(target[2]+distanceTol))) 
			{
				// then calculate the full Euclidean distance
				float distance = sqrt((node->point[0]-target[0])*(node->point[0]-target[0])
				 + (node->point[1]-target[1])*(node->point[1]-target[1])
				 + (node->point[2]-target[2])*(node->point[2]-target[2]));

				// if distance is within tolerance
				if (distance <= distanceTol)
					ids.push_back(node->id); // then add node to the list of nearby tree indices
			}
			// Compare the box boundary to current node to see if we want to flow left or right in the tree
			if ((target[depth%3]-distanceTol) < node->point[depth%3])
				searchHelper(target, node->left, depth+1, distanceTol, ids);
			if ((target[depth%3]+distanceTol) > node->point[depth%3])
				searchHelper(target, node->right, depth+1, distanceTol, ids);
		}
	}

	void insert(std::vector<float>& point, int id)
	{
		// Call a recursive helper function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		// pass memory address for root (starting point), because we will be wanting to reassign a NULL node in the tree
		//insertHelper(&root,0,point,id); 
		insertHelper(root,0,point,id);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids; // return indices of nearby points relative to the target point
	}
};

#endif