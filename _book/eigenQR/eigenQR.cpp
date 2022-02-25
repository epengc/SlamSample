#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;

#define MATRIX_SIZE 100 

int main(int argc, char** argv)
{
    Eigen::MatrixXd matrix_A;
    Eigen::MatrixXd vector_b;
    Eigen::MatrixXd QR_x;
    Eigen::MatrixXd Cholesky_x;

    matrix_A = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    matrix_A = matrix_A.transpose()*matrix_A;
    vector_b = Eigen::MatrixXd::Random(MATRIX_SIZE, 1);
    
    // QR Decompose
    QR_x = matrix_A.colPivHouseholderQr().solve(vector_b);

    // Cholesky Decompose
    Cholesky_x = matrix_A.ldlt().solve(vector_b);

    cout<<"QR_x = "<<endl<<QR_x<<endl;
    cout<<"Cholsesky_x = "<<endl<<Cholesky_x<<endl;

    return 0;
}

