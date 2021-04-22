/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
template <typename PointT>
struct Node
{
	std::vector<PointT> point;
	int id;
	Node<PointT>* left;
	Node<PointT>* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	Node(PointT arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

template <typename PointT>
struct KdTree
{
	Node<PointT> *root;

	KdTree(unsigned short d)
	: D(d), root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void place(Node<PointT> **n, std::vector<PointT> &point, int &id, uint depth) 
	{
		if(*n == NULL)
		{
			*n = new Node<PointT>(point, id);
		}
		else if(point[(int)(depth % D)] < (*n)->point[(int)(depth % D)])
		{
			place(&((*n)->left), point, id, ++depth);
		}
		else
		{
			place(&((*n)->right), point, id, ++depth);
		}
	}

	void insert(std::vector<PointT> point, int id)
	{
		// the function should create a new node and place correctly with in the root 
		place(&root, point, id, 0);
	}

	void insert(PointT point, int id)
	{
		// the function should create a new node and place correctly with in the root 
		place(&root, point, id, 0);
	}

	void digThrough(Node<PointT> **n, std::vector<PointT> *target, std::vector<int> *ids, float *tol, uint depth)
	{
		if(*n != NULL){
			if(abs((*target)[0] - (*n)->point[0]) <= *tol & abs((*target)[1] - (*n)->point[1]) <= *tol)
			{
				float dist = sqrt(pow((*target)[0] - (*n)->point[0], 2) + pow((*target)[1] - (*n)->point[1], 2));
				if (dist < *tol)
					(*ids).push_back((*n)->id);
			}

			uint cd = depth % D;
			if(((*target)[cd] - *tol) < (*n)->point[cd])
			{
				digThrough(&((*n)->left), target, ids, tol, ++depth);
			}
			if(((*target)[cd] + *tol) > (*n)->point[cd])
			{
				digThrough(&((*n)->right), target, ids, tol, ++depth);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<PointT> target, float distanceTol)
	{
		std::vector<int> ids;
		digThrough(&root, &target, &ids, &distanceTol, 0);
		return ids;
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		digThrough(&root, &target, &ids, &distanceTol, 0);
		return ids;
	}

	private:
		const unsigned short D{0};
};




