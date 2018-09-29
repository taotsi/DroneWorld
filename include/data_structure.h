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
	Point3D() {};
	Point3D(double x, double y, double z)
		: x_(x), y_(y), z_(z) {};
	double x_ = 0.0;
	double y_ = 0.0;
	double z_ = 0.0;
};
struct EulerAngle {
	EulerAngle() {};
	EulerAngle(double pitch, double yaw, double roll)
		: pitch_(pitch), yaw_(yaw), roll_(roll) {};
	double pitch_ = 0.0;
	double yaw_ = 0.0;
	double roll_ = 0.0;
};
struct Plane {
	Point3D p_start_;
	Point3D p_end_;
};
class PlaneWorld {
public:
	std::vector<Plane> data_;
};