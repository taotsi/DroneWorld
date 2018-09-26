#pragma once
#include <vector>

struct Pillar {
	float x_;
	float y_;
	float z1_;
	float z2_;
	float disp;
};
class PillarStixel{
public:
	std::vector<std::vector<Pillar>> data_;
};
class PillarCluster {
public:
	std::vector<std::vector<Pillar>> data_;
};

struct Point3D {
	float x_;
	float y_;
	float z_;
};
struct Plane {
	Point3D p_min_;
	Point3D p_max_;
};
class PlaneWorld {
public:
	std::vector<Plane> data_;
};