#include "3dTypes.h"

AABB::AABB(float minvalue, float maxvalue):  xmin(minvalue), ymin(minvalue), zmin(minvalue), xmax(maxvalue), ymax(maxvalue), zmax(maxvalue) {}

AABB::AABB(float* value) : xmin(*value), ymin(*(value + 1)), zmin(*(value + 2)), xmax(*(value + 3)), ymax(*(value + 4)), zmax(*(value + 5)) {}

AABB::AABB(float _xmin, float _ymin, float _zmin, float _xmax, float _ymax, float _zmax) : xmin(_xmin), ymin(_ymin), zmin(_zmin), xmax(_xmax), ymax(_ymax), zmax(_zmax) {}

float AABB::box_Area() {
	float a = xmax - xmin;
	float b = ymax - ymin;
	float c = zmax - zmin;
	return 2*(a * b + a * c + b * c);
}

bool compare_sort_param(const float &a, const float &b) {
	return a < b;
}

Primitive::Primitive(const vector<Point3d> &points_set){
	points = points_set;
	box = recompute_AABB();
}

float AABB::operator[](int i) {
	switch (i) {
		case 1: return xmin;
		case 2: return ymin;
		case 3: return zmin;
		case 4: return xmax;
		case 5: return ymax;
		case 6: return zmax;
	}
	return xmin;
};


AABB Primitive::recompute_AABB(){
	//auto minmax_x = std::minmax_element(blocks.begin(),blocks.end(), [](const Block& a, const Block& b) { return a.x < b.x; });	<- симпатичное, но более медленное через итератор. ћое решение делает всего 1 проход по массиву
	int t = points.size();
	if (t) {
		Point3d temp = points[0];
		float xmin, xmax;
		xmin = xmax = temp.x;
		float ymin, ymax;
		ymin = ymax = temp.y;
		float zmin, zmax;
		zmin = zmax = temp.z;
		for (int i = 1; i < t; i++) {
			temp = points[i];
			xmax = max(xmax, temp.x);
			xmin = min(xmin, temp.x);
			ymax = max(ymax, temp.y);
			ymin = min(ymin, temp.y);
			zmax = max(zmax, temp.z);
			zmin = min(zmin, temp.z);
		}
		return AABB(xmin, ymin, zmin, xmax, ymax, zmax);;
	}
	return NULL;
}
