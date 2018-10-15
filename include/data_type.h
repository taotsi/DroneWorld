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
    void Print(){
        std::cout << "( " << x_ << ", " << y_ << ", " << z_ << " )\n";
    }
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
    std::vector<double>& operator[](int pos){
        return data_[pos];
    }
	void PushStixel(std::vector<double> stixel) {
		data_.push_back(stixel);
	};
	void SetPosition(Point3D &position) {
		pos_camera_ = position;
	};
	void SetEulerAngle(Quaternion &quaternion) {
		angle_camera_ = EulerAngle(quaternion);
	};
    int size(){
        return static_cast<int>(data_.size());
    }
};
struct KdePeak {
	KdePeak() {};
    ~KdePeak() {};
	/* data */
	// pos in a scaled disparity frame
    int window_height_ = 2;
    double mean_ = 0.0;
    double right_ = 0.0;
    double left_ = 0.0;
    /* methods */
    void SetVal(double mean, double right, double left){
        mean_ = mean;
        if(right < left){
            auto temp = right;
            right = left;
            left = temp;
        }
        right_ = right;
        left_ = left;
    }
    void SetWindowHeight(int height){
        window_height_ = height > 1 ? height : 2;
    }
    void Print(){
        std::cout << "mean = " << mean_ << ", right = " << right_ << ", left = " << left_ << "\n";
    }
};

struct BlockedIndex {
    BlockedIndex(int range_length){
        index_.push_back(std::pair<int, int>(0, 0));
        index_.push_back(std::pair<int, int>(
            range_length-1, range_length-1));
    }
    BlockedIndex(int start, int end){
        index_.push_back(std::pair<int, int>(0, start));
        index_.push_back(std::pair<int, int>(end, end));
    }
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
    bool IsFull(){
        for(auto i=1; i<index_.size(); i++){
            if(index_[i-1].second < index_[i].first){
                return true;
            }
        }
        return false;
    };
    void Print(){
        for(auto seg : index_){
            std::cout << "(" << seg.first << ", " << seg.second << ")  ";
        }
        std::cout << std::endl;
    };
    std::pair<int, int>& operator[](unsigned int pos){
        if(pos >= index_.size()){
            std::cout << "BlockedIndex: index out of bounds" << std::endl;
            return index_[index_.size()-1];
        }
        return index_[pos];
    };
    int size(){
        return static_cast<int>(index_.size());
    }
};

enum FilterFlag{
    kCompliant,
    kNotCompliant,
    kEndPoint,
    kEndPointMaybe,
    kGap
};
struct FilterStatus{
    FilterStatus(FilterFlag &flag){
        flag_ = flag;
    }
    FilterStatus(FilterFlag &flag, int val){
        value_ = val;
        flag_ = flag;
    }
    int value_ = 0;
    FilterFlag flag_ = kCompliant;
};

class Pillar {
public:
    Pillar() {};
    Pillar(double x, double y, double z1, double z2)
        : x_(x), y_(y), z1_(z1), z2_(z2) {
        SortZ();
    }
    Pillar(Point3D p){
        x_ = p.x_;
        y_ = p.y_;
        z1_ = p.z_;
    }
    void SetPoint(Point3D p){
        x_ = p.x_;
        y_ = p.y_;
        z1_ = p.z_;
    }
    void SetXY(double x, double y){
        x_ = x;
        y_ = y;
    }
    void SetZ1(double z1){
        z1_ = z1;
        SortZ();
    }
    void SetZ2(double z2){ 
        z2_ = z2;
        SortZ(); 
    }
    void Print(){
        std::cout 
            << "( " << std::fixed << std::setw(7) << std::setprecision(3) << x_ 
            << ", " << std::fixed << std::setw(7) << std::setprecision(3) << y_ 
            << ", " << std::fixed << std::setw(7) << std::setprecision(3) << z1_ 
            << ", " << std::fixed << std::setw(7) << std::setprecision(3) << z2_ 
            << " )\n";
    }
    std::vector<double> GetCoor(){
        return std::vector<double>{x_, y_, z1_, z2_};
    }
    double x() { return x_; }
    double y() { return y_; }
    double z1() { return z1_; }
    double z2() { return z2_; }
private:
    /* data */
	double x_ = 0.0;
	double y_ = 0.0;
	double z1_ = 0.0;
	double z2_ = 0.0;
    /* methods */
    inline void SortZ(){
        if(z1_ > z2_){
            double temp = z1_;
            z1_ = z2_;
            z2_ = temp;
        }
    }
};

class SinglePillarCluster {
public:
    SinglePillarCluster(Pillar const &pillar){
        Push(pillar);
    }
    ~SinglePillarCluster() {};
    /* methods */
    Pillar& operator[](int pos) {
        return data_[pos];
    }
    void Push(Pillar pl) {
        data_.push_back(pl);
        if(!z1_vec_.empty()){
            z1_mean_ += (pl.z1()-z1_mean_)
                / static_cast<double>(z1_vec_.size()+1);
            z2_mean_ += (pl.z2()-z2_mean_)
                / static_cast<double>(z2_vec_.size()+1);
        }else{
            z1_mean_ = pl.z1();
            z2_mean_ = pl.z2();
        };
        z1_vec_.push_back(pl.z1());
        z2_vec_.push_back(pl.z2());
    }
    auto back() { return data_.back(); }
    int size(){ return static_cast<int>(data_.size()); }
    double z1_mean() { return z1_mean_; }
    double z2_mean() { return z2_mean_; }
    bool empty() { return data_.empty(); }
    double z_max() {
        return *(std::max_element(z2_vec_.begin(), z2_vec_.end()));
    };
    double z_min() {
        return *(std::min_element(z1_vec_.begin(), z1_vec_.end()));
    }
private:
    /* data */
    std::vector<Pillar> data_;
    // not a good design, redundant data. just in order to make fast
    std::vector<double> z1_vec_;
    std::vector<double> z2_vec_;
    double z1_mean_;
    double z2_mean_;
};

class PillarClusters {
public:
    PillarClusters() {};
    ~PillarClusters() {};
    /* data */
	std::vector<SinglePillarCluster> data_;
    /* methods */
    SinglePillarCluster& operator[](int pos) {
        return data_[pos];
    }
    // basic cluster
    void BringIn(Pillar pillar, double xy_max, double z_max) {
        if(!data_.empty()){
            bool is_brought_in = false;
            int n_cluster = static_cast<int>(data_.size());
            for(auto i=n_cluster-1; i>=0; i--){
                double dist = pow(data_[i].back().x() - pillar.x(), 2)
                    + pow(data_[i].back().y() - pillar.y(), 2);
                if(dist <= pow(xy_max, 2)){
                    if(fabs(data_[i].z1_mean()-pillar.z1()) <= z_max
                        && fabs(data_[i].z2_mean()-pillar.z2()) <= z_max){
                        data_[i].Push(pillar);
                        is_brought_in = true;
                        break;
                    }
                }
            }
            if(!is_brought_in){
                data_.push_back(SinglePillarCluster{pillar});
            }
        }else{
            data_.push_back(SinglePillarCluster{pillar});
        }
        return;
    }
    int size() {
        return static_cast<int>(data_.size());
    }
};
class Plane {
public:
    // for rectangle
    Plane() {};
    Plane(double x_min, double x_max, double y_min, 
        double y_max, double z_min, double z_max) {
            
    };
    // for rectangle
    Plane(Point3D p1, Point3D p2)
        : p1_(p1), p2_(p2) { };
    // for quadrilateral
    Plane(Point3D p1, Point3D p2, Point3D p3, Point3D p4)
        : p1_(p1), p2_(p2), p3_(p3), p4_(p4) {};
    ~Plane() {};
    /* data */
    std::vector<Plane*> left_ = std::vector<Plane*>{nullptr};
    std::vector<Plane*> right_ = std::vector<Plane*>{nullptr};
private:
    /* data */
    Point3D p1_;
	Point3D p2_;
    Point3D p3_;
	Point3D p4_;
};
class PlaneWorld {
public:
	std::vector<Plane> data_;
};

} // namespace droneworld