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
	Node *root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void place(Node **n, std::vector<float> &point, int &id, uint depth) 
	{
		if(*n == NULL)
		{
			*n = new Node(point, id);
		}
		else if(point[(int)(depth % 2)] < (*n)->point[(int)(depth % 2)])
		{
			place(&(*n)->left, point, id, ++depth);
		}
		else
		{
			place(&(*n)->right, point, id, ++depth);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// the function should create a new node and place correctly with in the root 
		place(&root, point, id, 0);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};




