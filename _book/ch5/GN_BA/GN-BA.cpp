//
// Created by xiang on 12/21/17.
//

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

#include "sophus/se3.hpp"

using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector2d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p3d_file = "./p3d.txt";
string p2d_file = "./p2d.txt";

int main(int argc, char **argv) {

    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d K;
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    // load points in to p3d and p2d 
    // START YOUR CODE HERE
    fstream p3d_reader(p3d_file);
    fstream p2d_reader(p2d_file);
    while(!p3d_reader.eof()){
        double x, y, z;
        p3d_reader>>x>>y>>z;
        Vector3d P3_d(x, y, z);
        p3d.push_back(P3_d);
    }
    while(!p2d_reader.eof()){
        double x, y;
        p2d_reader>>x>>y;
        Vector2d P2_d(x, y);
        p2d.push_back(P2_d);
    }
    cout<<"Data read complete!"<<endl;
    // END YOUR CODE HERE
    assert(p3d.size() == p2d.size());

    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    cout << "points: " << nPoints << endl;

    Sophus::SE3d T_esti; // estimated pose

    for (int iter = 0; iter < iterations; iter++) {

        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();

        cost = 0;
        // compute cost
        for (int i = 0; i < nPoints; i++) {
            // compute cost for p3d[I] and p2d[I]
            // START YOUR CODE HERE 
            Vector3d t_est = T_esti.translation();
            Matrix3d R_est = T_esti.rotationMatrix();
            // Convert p3d in Cw to p3d in Camera
            Vector3d P_cam = R_est*p3d[i] + t_est;
            // Convert p3d in Camera to p2d in Coordinate System of pixel
            Vector3d P_pixel = K*P_cam/P_cam.z();
            // Compute cost
            Vector2d e(p2d[i].x()-P_pixel.x(), p2d[i].y()-P_pixel.y());
            cost += e.transpose()*e;
	    // END YOUR CODE HERE

	    // compute jacobian
            Matrix<double, 2, 6> J;
            // START YOUR CODE HERE 
            J(0, 0) = fx/P_cam.z();
            J(0, 1) = 0;
            J(0, 2) = -fx*P_cam.x()/(P_cam.z()*P_cam.z());
            J(0, 3) = -fx*P_cam.x()*P_cam.y()/(P_cam.z()*P_cam.z());
            J(0, 4) = fx+fx*P_cam.x()*P_cam.x()/(P_cam.z()*P_cam.z());
            J(0, 5) = -fx*P_cam.y()/P_cam.z();
            J(1, 0) = 0;
            J(1, 1) = fy/P_cam.z();
            J(1, 2) = -fy*P_cam.y()/(P_cam.z()*P_cam.z());
            J(1, 3) = -fy-fy*P_cam.y()*P_cam.y()/(P_cam.z()*P_cam.z());
            J(1, 4) = fy*P_cam.x()*P_cam.y()/(P_cam.z()*P_cam.z());
            J(1, 5) = fy*P_cam.y()/P_cam.z();
            J = -J;
	    // END YOUR CODE HERE

            H += J.transpose() * J;
            b += -J.transpose() * e;
        }

	// solve dx 
        Vector6d dx;
        
        // START YOUR CODE HERE 
        dx = H.ldlt().solve(b);
        // END YOUR CODE HERE

        if (isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // update your estimation
        // START YOUR CODE HERE 
        Sophus::SE3d dT = Sophus::SE3d::exp(dx);
        T_esti = T_esti*dT;

        // END YOUR CODE HERE
        
        lastCost = cost;

        cout << "iteration " << iter << " cost=" << cout.precision(12) << cost << endl;
    }

    cout << "estimated pose: \n" << T_esti.matrix() << endl;
    return 0;
}
