#pragma once
#include <math.h>
#include <vector>
#include <algorithm>

static int a = 100;

using namespace std;

struct Point3d {
	float x, y, z;
	Point3d(float value) : x(value), y(value), z(value) {}
	Point3d(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
	Point3d() {};
};


struct Vec3d {
	float x, y, z;
	Vec3d() {};
	Vec3d(float value) : x(value), y(value), z(value) {}
	Vec3d(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
	Vec3d(Point3d start, Point3d end) : x(end.x - start.x), y(end.y - start.y), z(end.z - start.z) {}
};

template <typename T = float, typename T1 = float, typename T2 = float, typename T3 = float>

struct Vec4d {
	T x, y, z, w;
	Vec4d() {};
	Vec4d(Vec3d v, T p) : x(v.x), y(v.y), z(v.z), w(p) {}
	Vec4d(T _x, T1 _y, T2 _z, T3 _w) : x(_x), y(_y), z(_z), w(_w) {};
	Vec4d(T _v) : x(_v), y(_v), z(_v), w(_v) {};
};


struct AABB {
	float xmin, ymin, zmin, xmax, ymax, zmax;
	
	AABB() {}
	AABB(float minvalue, float maxvalue);
	AABB(float* value);
	AABB(float _xmin, float _ymin, float _zmin, float _xmax, float _ymax, float _zmax);

	float box_Area();
	float operator[](int i);

};


struct Primitive {
	vector<Point3d> points;
	AABB box;

	Primitive(){}
	Primitive(const vector<Point3d> &points);

	AABB recompute_AABB();
	//при каждом изменении параметров примитива происходит пересчет его ограничивающего бокса
};

struct Axe_Min_Compare {
	char d;
	bool operator()(Primitive& a, Primitive& b) { return a.box[d] < b.box[d]; }
	Axe_Min_Compare(char _dim) : d(_dim) {};
};


struct Axe_Max_Compare {
	char d;
	bool operator()(Primitive& a, Primitive& b) { return a.box[d] < b.box[d]; }
	Axe_Max_Compare(char _dim) : d(_dim + 3) {};
};