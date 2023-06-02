#ifndef NAVEGATION_UTILS_H
#define NAVEGATION_UTILS_h

// Include libaries for matix operations
#include <Eigen/Core>
#include <Eigen/LU>
#include <cmath>

class WOMBAT_Kinematics {
    private:
    
        //Kinematic parameters
        double r;
        double d;
        double h;
        
        // Matrices' coeficients
        double phi_coef;
        double d_coef[2];

    public:
        
        // WOMBAT_Kinematics constructor
        WOMBAT_Kinematics(double* kinematic_const);
        
        // Compute matrices for kinematics
        Eigen::Matrix<double, 2, 2> compute_D(double theta);
        Eigen::Matrix<double, 1, 2> compute_Phi(double theta);
        Eigen::Matrix<double, 3, 2> compute_A(double theta);
};

class Kalman {
    private:

        // Kalman matrices
        Eigen::Matrix<double, 3, 3> Q;
        Eigen::Matrix<double, 3, 3> R;
        Eigen::Matrix<double, 3, 3> Ri;
        Eigen::Matrix<double, 3, 3> H;
        Eigen::Matrix<double, 3, 3> Ht;
        Eigen::Matrix<double, 3, 3> P;
        Eigen::Matrix<double, 3, 3> K;
        Eigen::Matrix<double, 3, 2> B;
        Eigen::Matrix<double, 3, 1> Z;
        Eigen::Matrix<double, 3, 1> x_hat;

    public:
        
        // Kalman constructor
        Kalman(Eigen::Matrix<double, 3, 3>& Q_,
               Eigen::Matrix<double, 3, 3>& R_,
               Eigen::Matrix<double, 3, 3>& H_,
               double* kns_params);

        // Make Kalman new estimation
        Eigen::Matrix<double, 3, 1> estimate(Eigen::Matrix<double, 3, 1>& q, Eigen::Matrix<double, 2, 1>& u, double dt);
        
};

class Odometry {
    private:
    
        // Kinematic equations
        WOMBAT_Kinematics* kh;

        // Current odometry states
        Eigen::Matrix<double, 3,1> q;

    public:

        // Odometry constructor and destructor
        Odometry(double r, double d, double h);
        ~Odometry();

        // Calculate odometry for timestep
        Eigen::Matrix<double, 3, 1> get_odom(Eigen::Matrix<double, 2, 1> u, double dt);       
};



namespace utils {
   
    // Euler angles and quaternion conversions
    double* euler_to_quat(double roll, double pitch, double yaw);
    double* quat_to_euler(double w, double x, double y, double z);
   
}

#endif
