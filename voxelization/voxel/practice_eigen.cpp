#include <eigen3/Eigen/Dense>
#include<iostream>
#include<eigen3/Eigen/Eigenvalues>
#include <chrono>
#include<iterator>
#include<algorithm>
#include<Eigen/Dense>


using namespace Eigen;
using namespace std;
using namespace std::chrono;

void Eig()
{
 //Matrix3d A;
 Matrix<long double, Dynamic, Dynamic> A(3,3);
 A << 0, 2, 3, 4, 5, 6, 7, 8, 9;
 cout << "Here is a 3x3 matrix, A:" << endl << A << endl << endl;
  std::cout << " A in array format "  << std:: endl;
  //std::cout<< " Norm in column 1 is " << A(all,2).norm() << std:: endl ; 
  std::cout<< " Norm in column 1 is " << A.col(2).norm() << std:: endl ; 
  //std::cout << A.col(0).array()-1 << std::endl ; 
  //A.col(0) = A.col(0).array()-1;
  
  std::cout << A << std::endl;
  std::cout << " The mean of column 1" << A.col(0).array().mean() << std::endl;

/*
for(auto vec : A.array().colwise())
{

double std_dev = std::sqrt((vec - vec.mean()).square().sum()/(vec.size()));
cout << "Here is the std:\n" << std_dev << "\n";

}

 */

//auto start = high_resolution_clock::now();



 EigenSolver< Matrix<long double, Dynamic, Dynamic> > es(A);
 
 Matrix<long double, Dynamic, Dynamic> D = es.pseudoEigenvalueMatrix();
 //Matrix3d V = es.pseudoEigenvectors();
 //auto stop = high_resolution_clock::now();
 //auto duration = duration_cast<microseconds>(stop - start);
 //cout << "Time taken by PCA function: "
  //       << duration.count() << " microseconds" << endl;

 cout << "The pseudo-eigenvalue matrix D is:" << endl << D << endl;
 //cout << " The sum over diagnal is : " << D.cwiseAbs().diagonal().sum() << endl ; 
// cout << "The pseudo-eigenvector matrix V is:" << endl << V << endl;
 
 //cout << "Finally, V * D * V^(-1) = " << endl << V * D * V.inverse() << endl;

int col_index, row_index;
cout << D.maxCoeff(&row_index, &col_index) << endl;
cout << row_index << " " << col_index << endl;
cout<< 1/2 << endl;

auto start = high_resolution_clock::now();
JacobiSVD<Matrix<long double, Dynamic, Dynamic>> svd(A);
Matrix<long double, Dynamic, Dynamic> D1 =svd.singularValues();
auto stop = high_resolution_clock::now();
 auto duration = duration_cast<microseconds>(stop - start);
 cout << "Time taken by PCA function: "
      << duration.count() << " microseconds" << endl;
cout << "Its singular values are:" << endl << D1 << endl;

/*
auto start = high_resolution_clock::now();
BDCSVD<MatrixXd> svd(A);
std::cout << svd.singularValues() << endl;
auto stop = high_resolution_clock::now();
auto duration = duration_cast<microseconds>(stop - start);
cout << "Time taken by PCA function: "
      << duration.count() << " microseconds" << endl;
      */

}
int main()
{

 Eig();
 
}