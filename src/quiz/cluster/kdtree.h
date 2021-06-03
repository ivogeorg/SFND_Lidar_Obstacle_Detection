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

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insert(std::vector<float> point, int id)
	{
		// DONE:: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly within the root 

		// NOTE:: Alternate coordinates (e.g. x-y-x-y-... for 2D) 
		// NOTE:: If instert coordinate larger, insert to the right, otherwise, to the left
		// NOTE:: For 2D, for id even (and 0) compare x, for odd compare y, using indices

		if (root == NULL) {
			root = new Node(point, id);
			// std::cout << "Inserted " << id << "-{" << point[0] << ", " << point[1] << "} as root" << std::endl;
		} else {
			Node *curr = root;
			Node *next = NULL;
			int coord = id % 2;  // NOTE: 2D only; even is x, odd is y
			while (true) {
				next = point[coord] > curr->point[coord] ? curr->right : curr->left;
				if (next == NULL) {
					next = new Node(point, id);
					point[coord] > curr->point[coord] ? curr->right = next : curr->left = next;
					// std::cout << "Inserted " << id << "-{" << point[0] << ", " << point[1] << "}" << std::endl;
					break;
				} else {
					curr = next;
					coord = (coord + 1) % 2;
				}
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		// TODO:: Implement tree search for neighbor points
		// TODO:: Isn't this what is used in Euclidean Clustering?

		std::vector<int> ids;
		return ids;
	}
	

};




