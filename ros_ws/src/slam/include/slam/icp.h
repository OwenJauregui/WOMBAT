#ifndef SLAM_ICP_H
#define SLAM_ICP_H

// Include matrices from Eigen
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/LU>

// Include custom libraries
#include "slam/utils.h"
#include "slam/kd_tree.h"

class ICP {

    private:
    
    int max_iterations;
    double tolerance;
    bool map_started;
    Eigen::MatrixX2d past_cloud;
    Eigen::Matrix<double, 3, 3> odom_tf;    

    public:  
    
    ICP(int max_iterations = 20, double tolerance = 0.001);
    Eigen::Matrix3d best_fit_transform(const Eigen::MatrixX2d& a_mat, const Eigen::MatrixX2d& b_mat);
    Eigen::MatrixX2d nearest_neighbor(const Eigen::MatrixX2d& src, KD_Tree& dst);
    Eigen::MatrixX2d nearest_neighbor(const Eigen::MatrixX2d& src, const Eigen::MatrixX2d& dst);
    Eigen::Matrix<double, 3, 3> icp(Eigen::MatrixX2d& a_mat, Eigen::MatrixX2d& b_mat, Eigen::Matrix3d& origin);
    Eigen::Matrix<double, 3, 3> icp(Eigen::MatrixX2d& a_mat);
};
    
#endif
