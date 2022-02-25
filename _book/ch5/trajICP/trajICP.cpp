#include <iostream>
#include <string>
#include <vector>
#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include <pangolin/var/var.h>
#include <pangolin/var/varextra.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/display/widgets.h>
#include <pangolin/display/default_font.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

string compare_file = "./compare.txt";
typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> PosesTypes;

void Data_reader(string path, PosesTypes & estimate, PosesTypes & groundTruth);
void DrawTrajectory(PosesTypes est, PosesTypes gd, string name);

int main()
{
    // To Do read file of compare.txt
    PosesTypes estimate, groundTruth;
    Data_reader(compare_file, estimate, groundTruth);
    DrawTrajectory(estimate, groundTruth, "Original");
    // To Do compute R and t by using ICP method
    Eigen::Vector3d est_mid(0, 0, 0);
    Eigen::Vector3d gd_mid(0, 0, 0);
    for(size_t i=0; i<estimate.size(); ++i){
        est_mid += estimate[i].translation();
        gd_mid += groundTruth[i].translation();
    }
    est_mid = est_mid/estimate.size();
    gd_mid = gd_mid/groundTruth.size();

    vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> est_remove_mid;
    vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> gd_remove_mid;

    for(size_t i=0; i<estimate.size(); ++i){
        est_remove_mid.push_back(estimate[i].translation()-est_mid);
        gd_remove_mid.push_back(groundTruth[i].translation()-gd_mid);
    }
    
    cout<<"est_remove_mid: "<<est_remove_mid.size()<<endl;
    cout<<"gd_remove_mid: "<<gd_remove_mid.size()<<endl;
    cout<<"Remove mid_point completes "<<endl;
    Eigen::Matrix3d W;
    W.setZero();
    for(size_t i=0; i<estimate.size(); ++i){
        W += gd_remove_mid[i]*est_remove_mid[i].transpose();
    }
    cout<<"W matrix is: "<<W<<endl;
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixU()*svd.matrixV().transpose();
    cout<<"Rotation matrix is: "<< R <<endl;
    // p' = Rp +t
    Eigen::Vector3d t = gd_mid-R*est_mid;
    cout<<"Translation Vector is: "<<t.transpose()<<endl;
    // To Do visualize the result
    /*
     *              [R.inverse(), R.transpose()*t]
     *  T_trans =   [0          ,           1    ]
     */
    Sophus::SE3d gd_to_cam_point;
    Eigen::Matrix3d R_inv = R.inverse();
    Eigen::Vector3d t_inv = -R.transpose()*t;

    PosesTypes gd_to_cam;

    for(size_t i=0; i<estimate.size(); ++i){
        gd_to_cam_point = Sophus::SE3d(R_inv, t_inv)*groundTruth[i];
        gd_to_cam.push_back(gd_to_cam_point);
    }

    DrawTrajectory(gd_to_cam, estimate, "trajectory");

    return 0;
}


void Data_reader(string path, PosesTypes & estimate, PosesTypes & groundTruth)
{
    PosesTypes poses;
    fstream file_reader(path);

    double time_, tx, ty, tz, qx, qy, qz, qw;
    Eigen::Vector3d t;
    Eigen::Vector3d v;
    Eigen::Quaterniond q;
    Eigen::Matrix3d Rotation_matrix;
    Sophus::SE3d SE3_from_Eigen;

    while(!file_reader.eof()){

        file_reader >> time_ >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        t<<tx, ty, tz;
        v<<qx, qy, qz;
        q.vec()=v;
        q.w()=qw;
        q.normalize();
        Rotation_matrix=q.toRotationMatrix();
        SE3_from_Eigen.setRotationMatrix(Rotation_matrix);
        SE3_from_Eigen.translation().x()=t.x();
        SE3_from_Eigen.translation().y()=t.y();
        SE3_from_Eigen.translation().z()=t.z();
        estimate.push_back(SE3_from_Eigen);
        
        file_reader >> time_ >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        t<<tx, ty, tz;
        v<< qx, qy, qz;
        q.vec()=v;
        q.w()=qw;
        q.normalize();
        Rotation_matrix=q.toRotationMatrix();
        SE3_from_Eigen.setRotationMatrix(Rotation_matrix);
        SE3_from_Eigen.translation().x() = t.x();
        SE3_from_Eigen.translation().y() = t.y();
        SE3_from_Eigen.translation().z() = t.z();
        groundTruth.push_back(SE3_from_Eigen);
    }
    cout<<"File read is completed."<<endl;
}


 void DrawTrajectory(PosesTypes est, PosesTypes gd, string name) {
     // create pangolin window and plot the trajectory
     pangolin::CreateWindowAndBind(name, 1024, 768);
     glEnable(GL_DEPTH_TEST);
     glEnable(GL_BLEND);
     glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
 
     pangolin::OpenGlRenderState s_cam(
             pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
             pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
     );
 
     pangolin::View &d_cam = pangolin::CreateDisplay()
             .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
             .SetHandler(new pangolin::Handler3D(s_cam));
 
 
     while (pangolin::ShouldQuit() == false) {
         glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
 
         d_cam.Activate(s_cam);
         glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
 
         glLineWidth(2);
         
         for (size_t i = 0; i < est.size() - 1; i++) {
             glColor3f(1.0f, 0.0f, 0.0f);
             glBegin(GL_LINES);
             auto p1 = est[i], p2 = est[i + 1];
             glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
             glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
             glEnd();
         }
 
         for (size_t i = 0; i < gd.size() - 1; i++) {
             glColor3f(0.0f, 1.0f, 0.0f);
             glBegin(GL_LINES);
             auto p1 = gd[i], p2 = gd[i + 1];
             glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
             glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
             glEnd();
         }
         pangolin::FinishFrame();
         usleep(5000);   // sleep 5 ms
     }
 }
