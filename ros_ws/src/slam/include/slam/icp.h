#ifndef SLAM_ICP_H
#define SLAM_ICP_H

// Include matrices from Eigen
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/LU>

// Inlcude utils
#include "slam/utils.h"

class ICP {

    private:
    
    int max_iterations;
    double tolerance;
    bool map_started;
    Eigen::MatrixX2d past_cloud;
    Eigen::Matrix<double, 3, 3> odom_tf;    

    public:  
    
    ICP(int max_iterations = 20, double tolerance = 0.001);
    Eigen::Matrix3d best_fit_transform(Eigen::MatrixX2d& a_mat, Eigen::MatrixX2d& b_mat);
    Eigen::MatrixX2d nearest_neighbor(Eigen::MatrixX2d& src, Eigen::MatrixX2d& dst);
    Eigen::Matrix<double, 3, 3> icp(Eigen::MatrixX2d& a_mat, Eigen::MatrixX2d& b_mat, Eigen::Matrix3d& origin);
    Eigen::Matrix<double, 3, 3> icp(Eigen::MatrixX2d& a_mat);
};
    
#endif
