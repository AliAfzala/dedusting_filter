#include <eigen3/Eigen/Dense>
#include<iostream>
#include<eigen3/Eigen/Eigenvalues>
#include <chrono>


using namespace Eigen;
using namespace std;
using namespace std::chrono;

void Eig()
{
 //Matrix3d A;
 MatrixXd A(3,3);
 A << 1, 2, 3, 4, 5, 6, 7, 8, 9;
 cout << "Here is a 3x3 matrix, A:" << endl << A << endl << endl;

  auto start = high_resolution_clock::now();


 EigenSolver<MatrixXd> es(A);
 
 Matrix3d D = es.pseudoEigenvalueMatrix();
 Matrix3d V = es.pseudoEigenvectors();
 auto stop = high_resolution_clock::now();
 auto duration = duration_cast<microseconds>(stop - start);
 cout << "Time taken by PCA function: "
         << duration.count() << " microseconds" << endl;

 cout << "The pseudo-eigenvalue matrix D is:" << endl << D << endl;
 cout << "The pseudo-eigenvector matrix V is:" << endl << V << endl;
 cout << "Finally, V * D * V^(-1) = " << endl << V * D * V.inverse() << endl;

int col_index, row_index;
cout << D.maxCoeff(&row_index, &col_index) << endl;
cout << row_index << " " << col_index << endl;

}
int main()
{

 Eig();
 
}