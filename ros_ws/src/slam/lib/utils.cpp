#include "slam/utils.h"

double utils::euclidean_distance(const Eigen::RowVectorXd& a, const Eigen::RowVectorXd& b)
{
    // Calculate the difference between the 2 vectors
    Eigen::VectorXd diff = b - a;

    // Calculate the distance
    double distance = sqrt(diff.array().square().sum());

    return distance;
}

Eigen::MatrixX2d utils::polar_to_cartesian(const Eigen::MatrixX2d& polar)
{
    // Declare the resulting coordinates
    Eigen::MatrixX2d cartesian(polar.rows(),2);
    
    // Compute x and y
    cartesian.col(0) = polar.col(0).array() * polar.col(1).array().cos();
    cartesian.col(1) = polar.col(0).array() * polar.col(1).array().sin();
    
    return cartesian; 
}

Eigen::VectorXd utils::linspace(double min, double max, int steps)
{
    // Calculate step size
    double step_size = (max-min)/steps;

    // Create linspace vector
    Eigen::VectorXd linspace(steps, 1);

    // Generate the linspace
    for(int i = 0; i < steps; i++) {
        linspace(i, 0) = min + i*step_size;
    }   

    return linspace;
}

double* utils::euler_to_quat(double roll, double pitch, double yaw)
{
    // Element abbreviation
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);

    double* q = new double[4]; 
    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;

    return q;
}

