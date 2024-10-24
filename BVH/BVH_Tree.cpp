#pragma once

#include "BVH_Tree.h"
#include <iostream>
#include <algorithm>
#include <stack>
#include <chrono>
#include <thread>
#include <mutex>


using namespace std;

void BVH_Tree::dir_walk(Node* root) {
	if (root) {
		cout << root << endl;
		dir_walk(root->left);
		dir_walk(root->right);
	}
};


void BVH_Tree::back_walk(Node* root) {
	if (root) {
		dir_walk(root->left);
		dir_walk(root->right);
		cout << root << endl;
	}
};


void BVH_Tree::symm_walk(Node* root) {
	if (root) {
		dir_walk(root->left);
		cout << root << endl;
		dir_walk(root->right);
	}
}


AABB BVH_Tree::recompute_box(Primitive* prim_list, size_t size) {
	float xmin, ymin, zmin;
	xmin = ymin = zmin = FLT_MAX;
	float xmax, ymax, zmax;
	xmax, ymax, zmax = -FLT_MAX;

	if (size > 0) {
		Primitive* temp;
		for (size_t offset = 0; offset < size; offset++) {
			temp = prim_list + offset;
			xmax = max(xmax, temp->box.xmax);
			xmin = min(xmin, temp->box.xmin);
			ymax = max(ymax, temp->box.ymax);
			ymin = min(ymin, temp->box.ymin);
			zmax = max(zmax, temp->box.zmax);
			zmin = min(zmin, temp->box.zmin);
		}
		return AABB(xmin, ymin, zmin, xmax, ymax, zmax);
	}
	else return NULL;
}


pair<float, int> BVH_Tree::minSah_onDim(Primitive* prim_list, size_t size, const float &start_sah) {
	//��-���������� ����� ������ �� O(2N) ����� ������� ������� ��� ��������� ���� ��������� ���������

	Point3d min_left_on_axe = FLT_MAX;
	Point3d max_right_on_axe = FLT_MIN;
	unsigned int num_in_leftB;

	vector<float> sah;
	sah.reserve(size);

	Primitive temp;
	AABB temp_box;
	float temp_sah;
	for (size_t offset = 0; offset < size - 1; offset++) {
		//������� ��������� �� �����! ��� ��� ������������� �� ����, ���� ������ �������� ����������� ����� �������������� ������ 
		temp = *(prim_list + size - offset - 1);
		temp_box = temp.box;
		min_left_on_axe.x = min(temp.box.xmin, min_left_on_axe.x);
		min_left_on_axe.y = min(temp.box.ymin, min_left_on_axe.y);
		min_left_on_axe.z = min(temp.box.zmin, min_left_on_axe.z);

		max_right_on_axe.x = max(temp.box.xmax, max_right_on_axe.x);
		max_right_on_axe.y = max(temp.box.ymax, max_right_on_axe.y);
		max_right_on_axe.z = max(temp.box.zmax, max_right_on_axe.z);

		temp_box.xmin, temp_box.ymin , temp_box.zmin = min_left_on_axe.x, min_left_on_axe.y, min_left_on_axe.z;
		temp_box.xmax, temp_box.ymax, temp_box.zmax = max_right_on_axe.x, max_right_on_axe.y, max_right_on_axe.z;

		sah.push_back(temp_box.box_Area());
		//����������� �� ���������, ��� ������� ��� ������� ����������� � ������ ���������
	}

	temp_sah = start_sah;			//������ �������� �� ������� sah
	min_left_on_axe = FLT_MAX;
	max_right_on_axe = FLT_MIN;
	size_t sah_size = sah.size();
	float counted_sah;

	for (size_t offset = 0; offset < size - 1; offset++) {
		//������� ��������� �� �����! ��� ��� ������������� �� ��������� ���, ���� ������ �������� �����������-������������ ����� �������������� ������ 
		temp = *(prim_list + offset);
		temp_box = temp.box;
		min_left_on_axe.x = min(temp.box.xmin, min_left_on_axe.x);
		min_left_on_axe.y = min(temp.box.ymin, min_left_on_axe.y);
		min_left_on_axe.z = min(temp.box.zmin, min_left_on_axe.z);

		max_right_on_axe.x = max(temp.box.xmax, max_right_on_axe.x);
		max_right_on_axe.y = max(temp.box.ymax, max_right_on_axe.y);
		max_right_on_axe.z = max(temp.box.zmax, max_right_on_axe.z);

		temp_box.xmin, temp_box.ymin, temp_box.zmin = min_left_on_axe.x, min_left_on_axe.y, min_left_on_axe.z;
		temp_box.xmax, temp_box.ymax, temp_box.zmax = max_right_on_axe.x, max_right_on_axe.y, max_right_on_axe.z;

		counted_sah = EMPTY_NODE_COST_TRAVERSE + temp_box.box_Area() * (offset + 1) + sah[sah_size - offset - 1] * (size - offset + 1);

		if (counted_sah < temp_sah) {
			temp_sah = counted_sah;
			num_in_leftB = offset + 1;
		}
		//����������� �� ���������, ��� ������� ��� ������� ����������� � ����� ���������
	}
	return make_pair(temp_sah, num_in_leftB);		//����� �� 2 ������� ��� ��� ��������� ����� ����������� sah
}


void BVH_Tree::build_r_ObjectSplit(Primitive* prim_list, size_t size) {
	root = new Node;
	build_ObjectSplit(root, prim_list, size);
}


void BVH_Tree::build_ObjectSplit(Node* root, Primitive* prim_list, size_t size) {		//�� ������ ����� ������ � KD-tree, ��� � � KD-tree ���� ������������ ������ �������, � �� ��� �������������� �����
	
	size_t partition_count_primitives = 1;
	//������� ������ �������� - ���� � ���������������  ��������� ������� ����� 1 ��������
	AABB round_box = recompute_box(prim_list, size);

	//������� ������ ��������������� �����
	root->box = round_box;

	//�� ������ ������ ����������� ������ ��������� �������������� ����, ������� ����� �������� � ���� �������� �����

	//������� ������� ����� ������ � �������� � Surface Area Heuristic. ����� �����������  ��������� �� ���� � ������������ ����������� SAH. ��� �������� ��� ���������� ��������� ������������ � ������� ������� ���������������
	float root_area = round_box.box_Area();
	float start_SAH = SAH_OVERSPLIT_TRESHOLD * root_area * size;

	char priority_dim = 0;		//������������ ��������� ��������� ������ ����, ���� ����������� SAH �� ����� ������. � ����� ������ �� �������� ���������
	pair<float, int> temp_partition;

	for (char dim = 1; dim < 4; dim++) {
		//��� ��� ���������� ���������� in-place, ����� ���������� ����������� ��������� �� ����������� ��� ����� ����� ������������� ��� ���!
		sort(prim_list, prim_list + size, Axe_Max_Compare(dim));
		temp_partition = minSah_onDim(prim_list, size, start_SAH);
		if (temp_partition.first < start_SAH) {
			start_SAH = temp_partition.first;
			partition_count_primitives = temp_partition.second;
			priority_dim = dim;
		}
	}
	if (priority_dim == 0) {
		//���� ��������� ������ ���, ������ ���� �������� � ������ � ���� ������� ���������
		root->prim = prim_list;
		root->prim_size = size;
		return;
	}
	sort(prim_list, prim_list + size, Axe_Max_Compare(priority_dim));
	
	//sort(prim_list, prim_list + size, [&](Primitive& a, Primitive& b) { return a[priority_dim] < b[priority_dim]; });

	Primitive* temp_left = prim_list;
	Primitive* temp_right = prim_list + partition_count_primitives;

	root->left = new Node;
	build_ObjectSplit(root->left, temp_left, partition_count_primitives);

	root->right = new Node;
	build_ObjectSplit(root->right, temp_right, size - partition_count_primitives);

	return;
}



// ���� ����������� ������ ������ ����������, ��� ������������� ��������, ��� ���� �������������� ����������� ��� ��������� ���������� ������
// � ��� ��� ������������� ������� object split ���� ����������� ����������� ��������� ��������������� ��� ����������� ������, ��� ��� ���������� ���������� � ������������� ���� �� ����� ������ ������
// � ����� ���� ����������� ������������� ����������� ��������� ������ � ���� move ��� �������� �� ��������� ������, ��� ���� ��� �������������� ��������� �� ������� ���������� �������.


void BVH_Tree::build_ObjectSplit_Iteratively(Primitive* prim_list, size_t size) {

	root = new Node;

	struct temp_args { Node* t_root; Primitive* t_prim_list; size_t t_size; };
	stack<temp_args> st;
	st.push({root, prim_list, size});

	while (!st.empty()) {

		temp_args temp = move(st.top());
		st.pop();

		auto t1 = chrono::high_resolution_clock::now();

		size_t partition_count_primitives = 1;
		//������� ������ �������� - ���� � ���������������  ��������� ������� ����� 1 ��������
		AABB round_box = recompute_box(temp.t_prim_list, temp.t_size);

		//������� ������ ��������������� �����
		temp.t_root->box = round_box;

		//�� ������ ������ ����������� ������ ��������� �������������� ����, ������� ����� �������� � ���� �������� �����

		//������� ������� ����� ������ � �������� � Surface Area Heuristic. ����� �����������  ��������� �� ���� � ������������ ����������� SAH. ��� �������� ��� ���������� ��������� ������������ � ������� ������� ���������������
		float root_area = round_box.box_Area();
		float start_SAH = SAH_OVERSPLIT_TRESHOLD * root_area * temp.t_size;

		char priority_dim = 0;		//������������ ��������� ��������� ������ ����, ���� ����������� SAH �� ����� ������. � ����� ������ �� �������� ���������
		pair<float, int> temp_partition;

		for (char dim = 1; dim < 4; dim++) {
			//��� ��� ���������� ���������� in-place, ����� ���������� ����������� ��������� �� ����������� ��� ����� ����� ������������� ��� ���!
			sort(temp.t_prim_list, temp.t_prim_list + temp.t_size, Axe_Max_Compare(dim));
			temp_partition = minSah_onDim(temp.t_prim_list, temp.t_size, start_SAH);
			if (temp_partition.first < start_SAH) {
				start_SAH = temp_partition.first;
				partition_count_primitives = temp_partition.second;
				priority_dim = dim;
			}
		}
		if (priority_dim == 0) {
			//���� ��������� ������ ���, ������ ���� �������� � ������ � ���� ������� ���������
			temp.t_root->prim = temp.t_prim_list;
			temp.t_root->prim_size = temp.t_size;
			continue;
		}
		sort(temp.t_prim_list, temp.t_prim_list + temp.t_size, Axe_Max_Compare(priority_dim));


		auto t2 = chrono::high_resolution_clock::now();
		auto ms_int = chrono::duration_cast<chrono::microseconds>(t2 - t1);
		cout << ms_int.count() << endl;


		Primitive* temp_left = temp.t_prim_list;
		Primitive* temp_right = temp.t_prim_list + partition_count_primitives;

		temp.t_root->right = new Node;
		st.push({ temp.t_root->right, temp_right, temp.t_size - partition_count_primitives });

		temp.t_root->left = new Node;
		st.push({ temp.t_root->left, temp_left, partition_count_primitives });
	}
}


/*

void BVH_Tree_Threaded::build_ObjectSplit_Multithreaded(Primitive* prim_list, size_t size, int workers) {
	//�������� ��� �� n ��������. � ��� ����� ����� ����, � ������� ��� ����� �������� "������" � ���������� ��� �������������
	//������ ���� ����� �� ����� ��������. � ������ ����, ����� ���� ��� �������� ���������� ����� ������, �������� ������ � ���������, ��������� �� �� ����� ��� ���� ��� ���������

	vector<thread> MainThreadPool;
	MainThreadPool.reserve(workers);
	Node* root = new Node;
	st.push({ root, prim_list, size });

}

void BVH_Tree_Threaded::RunSplittingThread() {
	mutex stack_mutex;
	bool end_thread = false;

	while (true) {
		stack_mutex.lock();
		if (st.empty()) {
			end_thread = true;
		}
		stack_mutex.unlock();
		if (end_thread) return;

		arguments temp = st.top();
		st.pop();

	}
}


void BVH_Tree_Threaded::build_ObjectSplit__Multithreaded(arguments &temp) {

	size_t partition_count_primitives = 1;
	//������� ������ �������� - ���� � ���������������  ��������� ������� ����� 1 ��������
	AABB round_box = recompute_box(temp.t_prim_list, temp.t_size);

	//������� ������ ��������������� �����
	temp.t_root->box = round_box;

	//�� ������ ������ ����������� ������ ��������� �������������� ����, ������� ����� �������� � ���� �������� �����

	//������� ������� ����� ������ � �������� � Surface Area Heuristic. ����� �����������  ��������� �� ���� � ������������ ����������� SAH. ��� �������� ��� ���������� ��������� ������������ � ������� ������� ���������������
	float root_area = round_box.box_Area();
	float start_SAH = SAH_OVERSPLIT_TRESHOLD * root_area * temp.t_size;

	char priority_dim = 0;		//������������ ��������� ��������� ������ ����, ���� ����������� SAH �� ����� ������. � ����� ������ �� �������� ���������
	pair<float, int> temp_partition;

	for (char dim = 1; dim < 4; dim++) {
		//��� ��� ���������� ���������� in-place, ����� ���������� ����������� ��������� �� ����������� ��� ����� ����� ������������� ��� ���!
		sort(temp.t_prim_list, temp.t_prim_list + temp.t_size, Axe_Max_Compare(dim));
		temp_partition = minSah_onDim(temp.t_prim_list, temp.t_size, start_SAH);
		if (temp_partition.first < start_SAH) {
			start_SAH = temp_partition.first;
			partition_count_primitives = temp_partition.second;
			priority_dim = dim;
		}
	}
	if (priority_dim == 0) {
		//���� ��������� ������ ���, ������ ���� �������� � ������ � ���� ������� ���������
		temp.t_root->prim = temp.t_prim_list;
		temp.t_root->prim_size = temp.t_size;
		return;
	}
	sort(temp.t_prim_list, temp.t_prim_list + temp.t_size, Axe_Max_Compare(priority_dim));

	Primitive* temp_left = temp.t_prim_list;
	Primitive* temp_right = temp.t_prim_list + partition_count_primitives;

	temp.t_root->right = new Node;
	//st.push({ temp.t_root->right, temp_right, temp.t_size - partition_count_primitives });

	temp.t_root->left = new Node;
	//st.push({ temp.t_root->left, temp_left, partition_count_primitives });

}



*/