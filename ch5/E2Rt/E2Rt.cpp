//
// Created by 高翔 on 2017/12/19.
// 本程序演示如何从Essential矩阵计算R,t
//

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

#include <sophus/so3.hpp>

#include <iostream>

using namespace std;

int main(int argc, char **argv) {

    // 给定Essential矩阵
    Matrix3d E;
    E << -0.0203618550523477, -0.4007110038118445, -0.03324074249824097,
            0.3939270778216369, -0.03506401846698079, 0.5857110303721015,
            -0.006788487241438284, -0.5815434272915686, -0.01438258684486258;

    // 待计算的R,t
    Matrix3d R;
    Vector3d t;

    // SVD and fix sigular values
    // START YOUR CODE HERE
    Eigen::JacobiSVD<Eigen::Matrix3d> SVD(E, Eigen::ComputeFullV | Eigen::ComputeFullU);
    Eigen::Matrix3d U = SVD.matrixU();
    Eigen::Matrix3d V = SVD.matrixV();

    Eigen::Matrix3d Sigma = U.inverse()*E*V.transpose().inverse();
    double sigma_1 = Sigma(0,0);
    double sigma_2 = Sigma(1,1);
    double sigma = (sigma_1+sigma_2)/2;
    Eigen::Matrix3d Sigma_hat = Eigen::Matrix3d::Zero();
    Sigma_hat(0,0) = sigma;
    Sigma_hat(1,1) = sigma;
    cout<<Sigma<<endl;

    // END YOUR CODE HERE

    // set t1, t2, R1, R2 
    // START YOUR CODE HERE
    Matrix3d t_wedge1;
    Matrix3d t_wedge2;

    Matrix3d R1;
    Matrix3d R2;
    // END YOUR CODE HERE
    Eigen::AngleAxisd Rz_position(M_PI/2, Eigen::Vector3d(0, 0, 1));
    Eigen::AngleAxisd Rz_minus(-M_PI/2, Eigen::Vector3d(0, 0, 1));
    R1 = U*Rz_position.toRotationMatrix()*V.transpose();
    R2 = U*Rz_minus.toRotationMatrix()*V.transpose();
    t_wedge1 = U*Rz_position.toRotationMatrix()*Sigma_hat*U.transpose();
    t_wedge2 = U*Rz_minus.toRotationMatrix()*Sigma_hat*U.transpose();

    cout << "R1 = " << R1 << endl;
    cout << "R2 = " << R2 << endl;
    cout << "t1 = " << Sophus::SO3d::vee(t_wedge1) << endl;
    cout << "t2 = " << Sophus::SO3d::vee(t_wedge2) << endl;

    // check t^R=E up to scale
    Matrix3d tR = t_wedge1 * R1;
    cout << "t^R = " << tR << endl;

    return 0;
}
