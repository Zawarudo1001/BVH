#include <iostream>
#include "3dTypes.h"
#include "BVH_Tree.h"
#include <algorithm>
#include <chrono>
#include <random>


#define OBJECTS_COUNT 7500


using namespace std;


random_device dev;
mt19937 rng(dev());
uniform_int_distribution<std::mt19937::result_type> dist_center(0, 10000);
uniform_int_distribution<std::mt19937::result_type> dist_points(0, 10);



int main()
{
	BVH_Tree Main_BVH;

	Main_BVH.back_walk(Main_BVH.root);
	Main_BVH.dir_walk(Main_BVH.root);
	Main_BVH.symm_walk(Main_BVH.root);

	Primitive* Primitives = new Primitive[OBJECTS_COUNT];

	for (int i = 0; i < OBJECTS_COUNT; i++) {
		float center = float(dist_center(rng));

		Primitives[i] = (vector<Point3d>{ { center + float(dist_points(rng)), center + float(dist_points(rng)), center + float(dist_points(rng)) }, 
										{ center + float(dist_points(rng)), center + float(dist_points(rng)), center + float(dist_points(rng)) },
										{ center + float(dist_points(rng)) , center + float(dist_points(rng)), center + float(dist_points(rng)) } });
	}
	cout << "Started building BVH..." << endl;

	auto t1 = chrono::high_resolution_clock::now();

	/* Getting number of microseconds as an integer. */
	//Main_BVH.build_r_ObjectSplit(Primitives, OBJECTS_COUNT);
	Main_BVH.build_ObjectSplit_Iteratively(Primitives, OBJECTS_COUNT);

	auto t2 = chrono::high_resolution_clock::now();
	auto ms_int = chrono::duration_cast<chrono::microseconds>(t2 - t1);
	cout << ms_int.count() << endl;

	Main_BVH.dir_walk(Main_BVH.root);

	delete[] Primitives;

	return 0;
}

