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

    void insertHelper(Node** node, uint depth, std::vector<float> point, int id){
        if (*node == NULL) {
            *node = new Node(point, id);
        } else {
           uint cd = depth%2;
           if (point[cd]<(*node)->point[cd]){
               insertHelper(&((*node)->left),depth+1,point,id);
           }else{
               insertHelper(&((*node)->right),depth+1,point,id);
           }

        }

    }
	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
        insertHelper(&root,0, point, id);
	}

    void searchHelper(const Node* const& node,const std::vector<float>& target, int depth, const float distanceTol, std::vector<int>& ids) const{
        if (node == nullptr) return;

        auto distance = [](std::vector<float> point,std::vector<float> target){
            return std::sqrt(std::pow(point[0]-target[0],2)+std::pow(point[1]-target[1],2));
        };

        uint cd = depth % 2;
        if (std::fabs(target[0]-node->point[0])<=distanceTol && std::fabs(target[1]-node->point[1])<=distanceTol){
            if (distance(node->point,target)<=distanceTol){ids.push_back(node->id);}
        }

        if ((target[cd]-distanceTol)<=node->point[cd]){searchHelper(node->left,target,depth+1,distanceTol,ids);}
        if ((target[cd]+distanceTol)>=node->point[cd]){searchHelper(node->right,target,depth+1,distanceTol,ids);}

    }

    // return a list of point ids in the tree that are within distance of target
	std::vector<int> search(const std::vector<float>& target, float distanceTol) const
	{
		std::vector<int> ids;
        searchHelper(root,target,0,distanceTol,ids);
		return ids;
	}
	

};




