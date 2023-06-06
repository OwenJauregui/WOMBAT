#ifndef SLAM_UTILS_H
#define SLAM_UTILS_H

#include <Eigen/Core>
#include <Eigen/LU>

namespace utils {
    double euclidean_distance(const Eigen::RowVectorXd& a, const Eigen::RowVectorXd& b);
    
    Eigen::MatrixX2d polar_to_cartesian(const Eigen::MatrixX2d& polar);

    Eigen::VectorXd linspace(double min, double max, int steps);
}

#endif
