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
          0, 0, 1, 0, 0, 1, 0, 1, 1,
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

  Eigen::SelfAdjointEigenSolver<MatrixXd> solver(L_g);
  Eigen::VectorXd eivals = solver.eigenvalues();
  cout << "The eigenvalues of the L_g matrix are:" << endl << eivals << endl;
  cout << "The eigenvectors of the L_g matrix are:" << endl << solver.eigenvectors() << endl;

  double  max_eigenvalue = eivals.maxCoeff();
  cout << "The largest eigenvalue is: " << max_eigenvalue << endl;  

  double  min_eigenvalue = eivals.minCoeff();
  cout << "The smallest eigenvalue is: " << min_eigenvalue << endl;


  // ---------
  // TASK 4
  // ---------

  double min = 100;
  int min_index = 0;
  for (int i = 0; i < eivals.size() ; ++i){
    if (eivals[i] < 1e-16) 
        continue;
    if (eivals[i] < min) {
      min = eivals[i];
      min_index = i;
    }
  }

  cout << "The second smallest eigenvalue is: " << min << endl;
  cout << "The corrisponding eigenvector is: " << endl << solver.eigenvectors().col(min_index) << endl;
  cout << L_g << endl;

  
  // ---------
  // TASK 5
  // ---------

  SparseMatrix<double> A_s;
  loadMarket(A_s, "social.mtx");
  cout << "The Frobenius norm of the matrix A_s is: " << A_s.norm() << endl;


  // ---------
  // TASK 6
  // ---------
  
  int n_s = A_s.rows();
  VectorXd v_s(n_s);
  for (int i = 0; i < n_s; i++) {
    int sum = 0;
    for (int j = 0; j < n_s; j++) {
      sum += A_s.coeff(i, j);
    }
    v_s[i] = sum;
  }

  MatrixXd D_s = v_s.asDiagonal();
  MatrixXd L_s = D_s - A_s;
  cout << "Number of non zero elements in L_s: " << L_s.nonZeros() << endl;
  // Checking if L_s is symmetric
  if((L_s - L_s.transpose()).norm() < 1e-16)
    cout << "L_s is symmetric" << endl;
  else
    cout << "L_s is not symmetric" << endl;
    

  // ---------
  // TASK 7
  // ---------

  /* We add a perturbation to the first diagonal entry
   * of L_s, export it to a .mtx file and use a LIS
   * solver to compute the largest eigenvalue of L_s */

  
  L_s(0, 0) += 0.2;
  if((L_s - L_s.transpose()).norm() < 1e-10) {
    cout << "perturbed L_s is symmetric" << endl;
  } else {
    cout << "perturbed L_s is not symmetric" << endl; 
  }
  SparseMatrix<double> L_s_sparse;
  L_s_sparse = L_s.sparseView();
  saveMarket(L_s, "L_s.mtx");
  int ret = system("./run_test.sh");
  int ret2 = system("./run_testshift.sh");
}
