#pragma once
#include "3dTypes.h"

#define SAH_OVERSPLIT_TRESHOLD 1.0f

#define EMPTY_NODE_COST_TRAVERSE 2.0f


struct Node {
	Node* left = nullptr;
	Node* right = nullptr;
	AABB box;
	Primitive* prim = nullptr;
	size_t prim_size = 0;
	Node(){}
	Node(AABB _box): box(_box) {};
};

struct temp_args { 
	Node* t_root = nullptr; 
	Primitive* t_prim_list = nullptr; 
	size_t t_size = 0; 
};


class BVH_Tree
{
public:
	Node* root = nullptr;

	BVH_Tree() {}
	~BVH_Tree() { destruct(root); }

	void build_ObjectSplit_Recursively(Primitive* prim_list, size_t size);

	void build_ObjectSplit(Node * root, Primitive * prim_list, size_t size);

	void build_ObjectSplit_Iteratively(Primitive* prim_list, size_t size);

	void build_ObjectSplit_Multithreaded(temp_args);

	void build_ObjectSplit_Multithreaded_start();

	AABB recompute_box(Primitive* prim_list, size_t size);

	pair<float, int> minSah_onDim(Primitive* prim_list, size_t size, const float &start_sah);		//returns minimal sah for one dim and num of nodes in left bound

	void search();

	void dir_walk(Node* root);

	void symm_walk(Node* root);

	void back_walk(Node* root);

	void destruct(Node* root) {
		if (root) {
			destruct(root->left);
			destruct(root->right);
			delete root;
		}
	}
};

class BVH_Multithreaded : BVH_Tree {



};

