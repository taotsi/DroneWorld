#pragma once
#include <vector>

struct Pillar {
	double x_;
	double y_;
	double z1_;
	double z2_;
	double disp;
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
	double x_;
	double y_;
	double z_;
};
struct Plane {
	Point3D p_min_;
	Point3D p_max_;
};
class PlaneWorld {
public:
	std::vector<Plane> data_;
};