#pragma once
#include<Eigen/Core>

class BoundingBox {
public:
	BoundingBox() {
        Vmax = -Eigen::Vector3f(std::numeric_limits<float>::infinity(),
            std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());
            Vmin = Eigen::Vector3f(std::numeric_limits<float>::infinity(),
                std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());
    }
	BoundingBox(Eigen::Vector3f max, Eigen::Vector3f min): Vmax(max), Vmin(min) {}
	Eigen::Vector3f Vmax;
	Eigen::Vector3f Vmin;
    
	bool intersect(Eigen::Vector3f ori, Eigen::Vector3f dir) {
        float eps = 0.000001f;
        Eigen::Vector3f invDir = Eigen::Vector3f(1.0f / dir.x(), 1.0f / dir.y(), 1.0f / dir.z());
        Eigen::Vector3f t_min;
        Eigen::Vector3f t_max;
        if (dir.x() >= 0) {
            t_min(0) = (Vmin.x() - ori.x()) * invDir.x();
            t_max(0) = (Vmax.x() - ori.x()) * invDir.x();
        }
        else {
            t_min(0) = (Vmax.x() - ori.x()) * invDir.x();
            t_max(0) = (Vmin.x() - ori.x()) * invDir.x();
        }
        if (dir.y() >= 0) {
            t_min(1) = (Vmin.y() - ori.y()) * invDir.y();
            t_max(1) = (Vmax.y() - ori.y()) * invDir.y();
        }
        else {
            t_min(1) = (Vmax.y() - ori.y()) * invDir.y();
            t_max(1) = (Vmin.y() - ori.y()) * invDir.y();
        }
        if (dir.z() >= 0) {
            t_min(2) = (Vmin.z() - ori.z()) * invDir.z();
            t_max(2) = (Vmax.z() - ori.z()) * invDir.z();
        }
        else {
            t_min(2) = (Vmax.z() - ori.z()) * invDir.z();
            t_max(2) = (Vmin.z() - ori.z()) * invDir.z();
        }
        float t_enter = std::max(t_min.x(), std::max(t_min.y(), t_min.z()));
        float t_exit = std::min(t_max.x(), std::min(t_max.y(), t_max.z()));

        if (t_enter <= t_exit && t_exit >= 0) return true;
        else return false;
    }
    Eigen::Vector3f Center() { return Vmax * 0.5 + Vmin * 0.5; }
    BoundingBox Union(BoundingBox other) {
        BoundingBox unionBox = BoundingBox();
        unionBox.Vmax(0) = std::max(Vmax.x(), other.Vmax.x());
        unionBox.Vmax(1) = std::max(Vmax.y(), other.Vmax.y());
        unionBox.Vmax(2) = std::max(Vmax.z(), other.Vmax.z());

        unionBox.Vmin(0) = std::min(Vmin.x(), other.Vmin.x());
        unionBox.Vmin(1) = std::min(Vmin.y(), other.Vmin.y());
        unionBox.Vmin(2) = std::min(Vmin.z(), other.Vmin.z());
        return unionBox;
    }
    Eigen::Vector3f distance() {return Eigen::Vector3f(Vmax(0)- Vmin(0), Vmax(1) - Vmin(1), Vmax(2) - Vmin(2)); }
};