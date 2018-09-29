#pragma once
#include <vector>

struct Point2D {
	Point2D() {};
	Point2D(double x, double y)
		: x_(x), y_(y) {};
	double x_ = 0.0;
	double y_ = 0.0;
};

struct ScaledDisparityFrame {
	std::vector<std::vector<double>> data_;
	Point2D pos_camera_;
	// TODO: add euler angle member
	void PushStixel(std::vector<double> stixel) {
		data_.push_back(stixel);
	};
};

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

struct KdePeak {
	KdePeak() {};
	KdePeak(double x, double y, int pos)
		:x_(x), y_(y), pos_(pos) {};
	double x_ = 0;
	double y_ = 0;
	// pos in a scaled disparity frame
	int pos_ = 0;
	void SetPos(int pos) { 
		pos_ = pos;
	};
};

struct Point2D {
	Point2D() {};
	Point2D(double x, double y)
		: x_(x), y_(y) {};
	double x_ = 0.0;
	double y_ = 0.0;
};

struct Point3D {
	Point3D() {};
	Point3D(double x, double y, double z)
		: x_(x), y_(y), z_(z) {};
	Point3D(Point2D p)
		:x_(p.x_), y_(p.y_), z_(0.0) {};
	double x_ = 0.0;
	double y_ = 0.0;
	double z_ = 0.0;
	void SetX(double x) { x_ = x; };
	void SetY(double y) { y_ = y; };
	void SetZ(double z) { z_ = z; };

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
	Point3D p1_;
	Point3D p2_;
};
class PlaneWorld {
public:
	std::vector<Plane> data_;
};