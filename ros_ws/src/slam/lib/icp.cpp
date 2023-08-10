#include "slam/icp.h"
#include <iostream>
#include <chrono>

ICP::ICP(int max_iterations, double tolerance)
{   
    // Assign values to constants and map
    this->past_cloud = Eigen::RowVector2d::Zero(1,2);
    this->max_iterations = max_iterations;
    this->tolerance  = tolerance;
    this->odom_tf    = Eigen::Matrix3d::Identity(3, 3); 
}

Eigen::Matrix3d ICP::best_fit_transform(const Eigen::MatrixX2d& a_mat, const Eigen::MatrixX2d& b_mat)
{
    // Calculate the centroid for each poincloud
    Eigen::RowVector2d centroid_A = a_mat.colwise().mean();
    Eigen::RowVector2d centroid_B = b_mat.colwise().mean();

    // Transpose matrices relative to the centroid
    Eigen::MatrixX2d a_a(a_mat.rows(), 2);
    Eigen::MatrixX2d b_b(b_mat.rows(), 2);

    a_a = a_mat.rowwise() - centroid_A;
    b_b = b_mat.rowwise() - centroid_B;

    // Compute the rotation matrix
    Eigen::Matrix2d h_mat = a_a.transpose() * b_b;
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(h_mat, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix2d u_mat  = svd.matrixU();
    Eigen::Matrix2d v_mat = svd.matrixV();

    Eigen::Matrix2d r_mat = v_mat * u_mat.transpose();
    
    // Special reflection case
    if(r_mat.determinant() < 0) {
        v_mat(1, 0) = -v_mat(1, 0);
        v_mat(1, 1) = -v_mat(1, 1);
        Eigen::Matrix2d r_mat = v_mat * u_mat.transpose();
    }
    
    // Translation matrix
    Eigen::Matrix<double, 2, 1> t_vec = centroid_B.transpose() - r_mat*centroid_A.transpose();
    
    // Complete transformation
    Eigen::Matrix3d matrix_T   = Eigen::Matrix3d::Identity(3, 3);
    matrix_T.block<2, 2>(0, 0) = r_mat;
    matrix_T.block<2, 1>(0, 2) = t_vec;

    return matrix_T;
}

Eigen::MatrixX2d ICP::nearest_neighbor(const Eigen::MatrixX2d& src, KD_Tree& dst)
{
    // Create output matrix
    Eigen::MatrixX2d nearest_neighbors(src.rows(), 2);
        
    auto start = std::chrono::high_resolution_clock::now();
    for(int i=0; i < src.rows(); i++) {
        // Check nearest neighbor for each point;
        nearest_neighbors.row(i) << dst.closest_point(src.row(i));
    }
    auto end = std::chrono::high_resolution_clock::now();
    
    std::cout << (end - start).count()/src.rows() << std::endl;
       
    return nearest_neighbors;
}
 
Eigen::MatrixX2d ICP::nearest_neighbor(const Eigen::MatrixX2d& src, const Eigen::MatrixX2d& dst)
{
    // Declare resulting matrix with structure [indx, distance]
    Eigen::MatrixX2d nearest_neighbors(src.rows(), 2);

    // Set index and minimum distance values
    int close_indx;
    double min_dist, cur_dist;    

    auto start = std::chrono::high_resolution_clock::now();

    // Find the closest point from dst for each point in src
    for(int i=0; i < src.rows(); i++) {

        // Initialize min distance to the first point
        close_indx = 0;
        min_dist   = utils::euclidean_distance(src.row(i), dst.row(0));
        
        // Calculate distance to each point in dst
        for(int j=1; j < dst.rows(); j++) {
            
            // Check if it is a new min distance
            cur_dist = utils::euclidean_distance(src.row(i), dst.row(j));
            
            if(cur_dist < min_dist) {
                // Update min distance and index for closest point
                min_dist = cur_dist;
                close_indx = j;
            }
        }

        // Save nearest neighbor into resulting matrix
        nearest_neighbors(i, 0) = close_indx;
        nearest_neighbors(i, 1) = min_dist;
    }

    auto end = std::chrono::high_resolution_clock::now();

    std::cout << (end - start).count()/src.rows() << std::endl;
    
    return nearest_neighbors;
}

Eigen::Matrix<double, 3, 3> ICP::icp(Eigen::MatrixX2d& a_mat, Eigen::MatrixX2d& b_mat, Eigen::Matrix3d& origin)
{
    // Create new point sets to better manipulate the mods
    Eigen::Matrix3Xd src(3, a_mat.rows());
    src.setOnes();

    // Copy a_mat and b_mat into the src and dst matrices
    src.block(0,0,2,src.cols()) = a_mat.transpose();

    // Declare and initialize the previous error
    double prev_error = 100;
    double mean_error;

    // Declare auxiliar matrices
    Eigen::Matrix3d t_matrix, total_t;
    Eigen::MatrixX2d neighbors, nearest_points;

    // Apply initial pose estimation 
    src     == origin*src;
    total_t =  origin;
    
    Eigen::MatrixX2d src_t(src.cols(),2);

    // Create a new kd-tree
    KD_Tree cloud_tree(b_mat);

    // Make the Closest Point iterations
    for(int i = 0; i < this->max_iterations; i++) {

        // Update source transposed
        src_t << (src.block(0,0,2,src.cols())).transpose();        
	// Declare and find nearest neighbors
        neighbors = this->nearest_neighbor(src_t, cloud_tree);
        std::cout<<b_mat.rows()<<std::endl;       

        // Select only nearest points from b_mat
        nearest_points = Eigen::MatrixX2d(neighbors.rows(), 2);    
        for(int row = 0; row < neighbors.rows(); row++) {
            nearest_points.row(row) = b_mat.row(neighbors(row, 0));
        }

        // Compute transform to nearest points
        t_matrix = best_fit_transform(src_t, nearest_points);
        total_t  = total_t*t_matrix;        
        
        std::cout << total_t << std::endl;

        // Update current source and calculate error
        src = t_matrix*src;
        mean_error = neighbors.col(1).mean();

        if(fabs(prev_error - mean_error) < this->tolerance) {
            break;
        } else {
            prev_error = mean_error;
        }
    }

    // Once tolerance or max iterations have been reached, calculate final transform
    // Eigen::MatrixX2d final_cloud = src.block(0,0,2,src.cols()).transpose();
    // t_matrix = best_fit_transform(a_mat, final_cloud);
    return total_t;
}

Eigen::Matrix<double, 3, 3> ICP::icp(Eigen::MatrixX2d& a_mat)
{
    Eigen::Matrix<double, 3, 3> matrix_T;

    if((this->past_cloud.array()).any()) {
        matrix_T = this->icp(a_mat, this->past_cloud, this->odom_tf);
        // this->odom_tf = matrix_T;
	
	// Generate new pointcloud transformed
	// Eigen::Matrix3Xd src(3, a_mat.rows());
	// src.setOnes();

        // Copy a_mat into the src matrix
        // src.block(0,0,2,src.cols()) = a_mat.transpose();
        // src = matrix_T*src;
	// this->past_cloud = src.block(0,0,2,src.cols()).transpose();
    } else {
        matrix_T = Eigen::Matrix3d::Identity(3, 3);
    }

    this->past_cloud = a_mat;
    
    return matrix_T;
}

