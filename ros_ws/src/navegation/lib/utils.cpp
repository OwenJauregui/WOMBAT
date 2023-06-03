#include "navegation/utils.h"

/*----------------------------------------------------------
         Start of WOMBAT_Kinematics implementation
----------------------------------------------------------*/

WOMBAT_Kinematics::WOMBAT_Kinematics(double r_, double d_, double h_)
{
    // Kinematic values
    this->r = r_;
    this->d = d_;
    this->h = h_;

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
               double r,
               double d,
               double h)
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
    this->B = WOMBAT_Kinematics(r, d, h).compute_A(0);
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
             Start of Odometry implementation
----------------------------------------------------------*/

Odometry::Odometry(double r, double d)
{   
    // Initialize states and create kinematics handler
    this->q << 0, 
               0, 
               0;

    this->kns_h = new WOMBAT_Kinematics(r, d, 0);
}

Odometry::~Odometry()
{
    // Delete kinematics handler
    delete this->kns_h;
}

Eigen::Matrix<double, 3, 1> Odometry::compute_odom(Eigen::Matrix<double, 2, 1>& u, double dt)
{
    // Calculate the kinematics for current states
    Eigen::Matrix<double, 3, 2> a_mat;
    a_mat = this->kns_h->compute_A(this->q(2, 0));

    // Calculate the new states
    this->q += dt*(a_mat*u);

    return this->q;
}

/*----------------------------------------------------------
              Start of Control implementation
----------------------------------------------------------*/

Control::Control(double k1, double k2, double r, double d)
{
    // Create kinematics handler
    this->kns_h =  new WOMBAT_Kinematics(r, d, 0);
    
    // Set control constants
    this->k     << k1, 0,
                   0, k2;
}

Control::~Control()
{
    delete this->kns_h;
}

Eigen::Matrix<double, 2, 1> Control::control_position(Eigen::Matrix<double, 3, 1>& q, Eigen::Matrix<double, 2, 1>& qd, double dt)
{
    // Calculate the state difference to the desired states
    Eigen::Matrix<double, 2, 1> qe = q.block(0,0,2,1) - qd;
    
    // Obtain the D matrix
    Eigen::Matrix<double, 2, 2> d_mat = kns_h->compute_D(q(2,0));

    // Compute the control
    Eigen::Matrix<double, 2, 1> u;
    u = d_mat*(-this->k)*qe;

    return u;
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

double* utils::quat_to_euler(double w, double x, double y, double z)
{
    // Euler angles array
    double* euler = new double[3];

    // Roll (x-axis rotation)
    double sinr_cosp = 2 * (w*x + y*z);
    double cosr_cosp = 1 - 2 * (x*x + y*y);
    euler[0] = atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = sqrt(1 + 2 * (w*y - x*z));
    double cosp = sqrt(1 - 2 * (w*y - x*z));
    euler[1] = 2 * atan2(sinp, cosp) - M_PI / 2;

    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (w*z + x*y);
    double cosy_cosp = 1 - 2 * (y*y + z*z);
    euler[2] = atan2(siny_cosp, cosy_cosp);

    return euler;
}
