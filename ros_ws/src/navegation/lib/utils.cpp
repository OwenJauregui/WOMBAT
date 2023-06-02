#include "navegation/utils.h"

/*----------------------------------------------------------
         Start of WOMBAT_Kinematics implementation
----------------------------------------------------------*/

WOMBAT_Kinematics::WOMBAT_Kinematics(double* kinematic_const)
{
    // Kinematic values
    this->r = kinematic_const[0];
    this->d = kinematic_const[1];
    this->h = kinematic_const[2];

    // Phi matrix coeficients
    this->phi_coef = this->r/this->d;

    // D matrix coeficients
    this->d_coef[0] = this->r/2;
    this->d_coef[1] = this->h*this->r/this->d;
}

Eigen::Matrix<double, 2, 2> WOMBAT_Kinematics::compute_D(double theta)
{
    // Compute the sin and cos for the given angle
    double s = sin(theta);
    double c = cos(theta);

    // Declare the D kinematics matrix 
    Eigen::Matrix<double, 2, 2> d_mat;

    // Initilize the D kinematics matrix
    d_mat(0, 0) = this->d_coef[0]*c - this->d_coef[1]*s;
    d_mat(0, 1) = this->d_coef[0]*c + this->d_coef[1]*s;
    d_mat(1, 0) = this->d_coef[0]*s + this->d_coef[1]*c;
    d_mat(1, 1) = this->d_coef[0]*s - this->d_coef[1]*c;

    return d_mat;
}

Eigen::Matrix<double, 1, 2> WOMBAT_Kinematics::compute_Phi(double theta)
{
    // Declare the Phi kinematics matrix 
    Eigen::Matrix<double, 1, 2> phi_mat;
    
    // Initialize matrix
    phi_mat << this->phi_coef, -this->phi_coef;
    
    return phi_mat;
}

Eigen::Matrix<double, 3, 2> WOMBAT_Kinematics::compute_A(double theta)
{
    // Declare the A kinematics matrix 
    Eigen::Matrix<double, 3, 2> a_mat;

    // Compute D and Phi and concatenate them
    a_mat << this->compute_D(theta), 
             this->compute_Phi(theta);

    return a_mat;
}

/*----------------------------------------------------------
              Start of Kalman implementation
----------------------------------------------------------*/

Kalman::Kalman(Eigen::Matrix<double, 3, 3>& Q_,
               Eigen::Matrix<double, 3, 3>& R_,
               Eigen::Matrix<double, 3, 3>& H_,
               double* kns_params)
{
    // Matrices given
    this->Q  = Q_;
    this->R  = R_;
    this->Ri = R_.inverse();
    this->H  = H_;
    this->Ht = H_.transpose();
    this->P  << 0, 0, 0,
                0, 0, 0,
                0, 0, 0;
    this->Z  << 0,
                0,
                0;
    this->x_hat = this->Z;    

    // Calculated matrices
    this->K  = this->P*this->Ht*this->Ri;

    // Create new kinematic equations handler
    this->B = WOMBAT_Kinematics(kns_params).compute_A(0);
}

Eigen::Matrix<double, 3, 1> Kalman::estimate(Eigen::Matrix<double, 3, 1>& q, Eigen::Matrix<double, 2, 1>& u, double dt)
{
    // Make estimations
    this->x_hat += dt*(this->B*u + this->K*(this->Z - this->H*this->x_hat));
    this->P += dt*(this->Q - this->K*this->H*this->P);

    // Update Z and K
    this->Z = this->H*q;
    this->K = this->P*this->Ht*this->Ri;

    return this->x_hat;
}

/*----------------------------------------------------------
          Start of utils namespace implementation
----------------------------------------------------------*/


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
