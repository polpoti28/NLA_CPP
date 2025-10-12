#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>
#include <unsupported/Eigen/SparseExtra>
#include "utils.h"

using std::cout;
using std::endl;
using Eigen::MatrixXd;
using Eigen::SparseMatrix;
using Eigen::VectorXd;

int main() {
  int n = 9;

  // ---------
  // TASK 1
  // ---------

  MatrixXd A_g(n,n);
  A_g <<  0, 1, 0, 1, 0, 0, 0, 0, 0,
          1, 0, 1, 0, 0, 0, 0, 0, 0,
          0, 1, 0, 1, 1, 0, 0, 0, 0,
          1, 0, 1, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 1, 0, 1, 1,
          0, 0, 0, 0, 1, 0, 1, 0, 0,
          0, 0, 0, 0, 0, 1, 0, 1, 1,
          0, 0, 0, 0, 1, 0, 1, 0, 1,
          0, 0, 0, 0, 1, 0, 1, 1, 0;

  cout << "The Frobenius norm of the matrix A_g is: " << A_g.norm() << endl;

  // ---------
  // TASK 2
  // ---------

  VectorXd v_g(n);
  for (int i = 0; i < n; i++) {
    int sum = 0;
    for (int j = 0; j < n; j++) {
      sum += A_g(i,j);
    }
    v_g[i] = sum;
  }

  MatrixXd D_g = v_g.asDiagonal();
  MatrixXd L_g = D_g - A_g;
  VectorXd x = VectorXd::Ones(n);
  VectorXd y = L_g * x;
  cout << y.norm() << endl;


  
  // ---------
  // TASK 3
  // ---------

  Eigen::EigenSolver<MatrixXd> solver(L_g);
  Eigen::VectorXd eivals = solver.eigenvalues().real();
  cout << "The eigenvalues of the L_g matrix are:" << endl << eivals << endl;
  cout << "The eigenvectors of the L_g matrix are:" << endl << solver.eigenvectors() << endl;

  double  max_eigenvalue = eivals.real().maxCoeff();
  cout << "The largest eigenvalue is: " << max_eigenvalue << endl;  

  double  min_eigenvalue = eivals.real().minCoeff();
  cout << "The smallest eigenvalue is: " << min_eigenvalue << endl;


  // ---------
  // TASK 4
  // ---------

  double min = 100;
  int min_index;
  for (int i = 0; i < eivals.size() ; ++i){
    if (eivals[i] < 1e-15) 
        continue;
    if (eivals[i] < min) {
      min = eivals[i];
      min_index = i;
    }
  }

  cout << "The second smallest eigenvalue is: " << min << endl;
  cout << "The related eigenvector is: " << endl << solver.eigenvectors().col(min_index) << endl;



}