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



class BVH_Tree
{
public:
	Node* root = nullptr;

	BVH_Tree() {}
	~BVH_Tree() { destruct(root); }

	void build_r_ObjectSplit(Primitive* prim_list, size_t size);

	void build_ObjectSplit(Node * root, Primitive * prim_list, size_t size);

	void build_ObjectSplit_Iteratively(Primitive* prim_list, size_t size);

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