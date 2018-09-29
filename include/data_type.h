#pragma once
#include <vector>
#include <cmath>

namespace droneworld{

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
	Point3D(Point2D p, double z)
		:x_(p.x_), y_(p.y_), z_(z) {};
	double x_ = 0.0;
	double y_ = 0.0;
	double z_ = 0.0;
};
struct Quaternion {
	Quaternion(double w, double x, double y, double z)
		:w_(w), x_(x), y_(y), z_(z) {};
	double w_;
	double x_;
	double y_;
	double z_;
};
struct EulerAngle {
	EulerAngle() {};
	EulerAngle(double pitch, double yaw, double roll)
		: pitch_(pitch), yaw_(yaw), roll_(roll) {};
	EulerAngle(Quaternion quat) {
		pitch_ = asin(2*quat.w_*quat.y_ - 2*quat.x_*quat.z_);
		yaw_ = atan2(2 * quat.w_*quat.z_,
			1 - 2 * quat.y_*quat.y_ - 2 * quat.z_*quat.z_);
		roll_ = atan2(2*quat.w_*quat.x_,
			1 - 2*quat.x_*quat.x_ - 2*quat.y_*quat.y_);

	};
	/* data */
	double pitch_ = 0.0;
	double yaw_ = 0.0;
	double roll_ = 0.0;
};
struct ScaledDisparityFrame {
	ScaledDisparityFrame() {};
	ScaledDisparityFrame(
		Point3D position, Quaternion quaternion) {
		SetPosition(position);
		SetEulerAngle(quaternion);
	}
	/* data */
	std::vector<std::vector<double>> data_;
	Point3D pos_camera_;
	EulerAngle angle_camera_;
	/* methods */
	void PushStixel(std::vector<double> stixel) {
		data_.push_back(stixel);
	};
	void SetPosition(Point3D position) {
		pos_camera_ = position;
	};
	void SetEulerAngle(Quaternion quaternion) {
		angle_camera_ = EulerAngle(quaternion);
	};
};
struct KdePeak {
	KdePeak() {};
	KdePeak(double x, double y, int pos) {
		p_world_.x_ = x;
		p_world_.y_ = y;
		pos_ = pos;
	};
	/* data */
	//Point2D p_camera_;
	Point2D p_world_;
	// pos in a scaled disparity frame
	int pos_ = 0;
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

struct Plane {
	Point3D p1_;
	Point3D p2_;
};
class PlaneWorld {
public:
	std::vector<Plane> data_;
};

} // namespace droneworld