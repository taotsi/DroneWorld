#pragma once
#include <vector>
#include <cmath>
#include <utility>
#include <algorithm>

// data types here are only responsible for storation,
// not calculations or algorithms. So make sure the data
// is correct before storing it.
 
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
	Point3D(Point2D &p, double z)
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
	EulerAngle(Quaternion &quat) {
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
		Point3D &position, Quaternion &quaternion) {
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
	void SetPosition(Point3D &position) {
		pos_camera_ = position;
	};
	void SetEulerAngle(Quaternion &quaternion) {
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
    KdePeak(Point2D p, int pos)
        : p_world_(p), pos_(pos) {};
	/* data */
	//Point2D p_camera_;
	Point2D p_world_;
	// pos in a scaled disparity frame
	int pos_ = 0;
    int window_left_ = 0;
    int window_right_ = 0;
    int windoe_height_ = 1;
    /* methods */
    void SetWindow(int left, int right, int height){
        window_left_ = left;
        window_right_ = right;
        windoe_height_ = height;
    };
};
struct BlockedIndex {
    /* data */
    std::vector<std::pair<int, int>> index_;
    /* methods */
    void AddSegment(int start, int end){
        if(start > end){
            std::cout << "wrong input for BlockedIndex.AddSegment, swapped start and end already\n";
            auto temp = start;
            start = end;
            end = temp;
        }
        int idx_l = 0;
        int idx_r = static_cast<int>(index_.size())-1;
        int idx_mid = (idx_l+idx_r) / 2;
        if(index_.empty()){
            index_.push_back(std::pair<int, int>(start, end));
            return;
        }else if(start < index_[0].first){
            index_.insert(index_.begin(), std::pair<int, int>(start, end));
            return;
        }else if(start > index_[index_.size()-1].first){
            index_.push_back(std::pair<int, int>(start, end));
            return;
        }else{
            while(idx_l < idx_r-1){
                if(start < index_[idx_mid].first){
                    idx_r = idx_mid;
                    idx_mid = (idx_l+idx_r) / 2;
                }else if(start > index_[idx_mid].first){
                    idx_l = idx_mid;
                    idx_mid = (idx_l+idx_r) / 2;
                }else{ // start == index_[idx_mid].first
                    idx_r = idx_mid;
                    break;
                }
            }
        }
        std::pair<int, int> pair;
        if(start >= index_[idx_r].first 
            || start < index_[idx_r-1].second){
            return;
        }else{
            if(end > index_[idx_r].first){
                // std::cout << "new segment'end(" << end << ") for BlockedIndex is bigger than next segment's start,  truncated it already\n";
                pair = std::pair<int, int>{start, index_[idx_r].first};
            }else{
                pair = std::pair<int, int>{start, end};
            }
        }
        index_.insert(index_.begin()+idx_r, pair);
    };
    void Print(){
        for(auto itr : index_){
            std::cout << "(" << itr.first << ", " << itr.second << ")  ";
        }
        std::cout << std::endl;
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

struct Plane {
	Point3D p1_;
	Point3D p2_;
};
class PlaneWorld {
public:
	std::vector<Plane> data_;
};

} // namespace droneworld