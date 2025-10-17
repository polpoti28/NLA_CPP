#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>
#include <fstream>
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
  SparseMatrix<double> D_s(n_s,n_s);
  for (int i = 0; i < n_s; i++) {
    double sum = 0;
    for (int j = 0; j < n_s; j++) {
      sum += A_s.coeff(i, j);
    }
    D_s.insert(i, i) = sum;
  }

  SparseMatrix<double> L_s = D_s - A_s;
  cout << "Number of non zero elements in L_s: " << L_s.nonZeros() << endl;
  // Checking if L_s is symmetric
  SparseMatrix<double> L_sT = SparseMatrix<double>(L_s.transpose());
  if((L_s - L_sT).norm() < 1e-16)
    cout << "L_s is symmetric" << endl;
  else
    cout << "L_s is not symmetric" << endl;
    

  // ---------
  // TASK 7
  // ---------

  /* We add a perturbation to the first diagonal entry
   * of L_s, export it to a .mtx file and use a LIS
   * solver to compute the largest eigenvalue of L_s */

  L_s.coeffRef(0, 0) += 0.2;
  SparseMatrix L_sT2 = SparseMatrix<double>(L_s.transpose());
  if((L_s - L_sT2).norm() < 1e-16) {
    cout << "perturbed L_s is symmetric" << endl;
  } else {
    cout << "perturbed L_s is not symmetric" << endl; 
  }
  saveMarket(L_s, "L_s.mtx");
  int ret = system("./run_test.sh");


  // ---------
  // TASK 8
  // ---------

  //int ret2 = system("./run_testshift.sh");


  // ---------
  // TASK 9
  // ---------

  //int ret3 = system("./run_test_inverse.sh");


  // ---------
  // TASK 9
  // ---------

  SparseMatrix<double> eigenVecs;
  loadMarket(eigenVecs, "eigvec_i.mtx");
  VectorXd eigVec = eigenVecs.col(1);

  int n_p(0), n_n(0);
  std::vector<int> clusterA;
  std::vector<int> clusterB;

  for (int i = 0; i < n_s; i++) {
    if (eigVec[i] < 0) {
      clusterB.emplace_back(i);
      n_n++;
    } else {
      clusterA.emplace_back(i);
      n_p++;
    }
  }


  cout << "Number of positive entries: " << n_p << endl;
  cout << "Number of negative entries: " << n_n << endl;

  
  // ---------
  // TASK 10
  // ---------

  SparseMatrix<double> P(n_s, n_s);
  for (int i = 0; i < n_p; i++) {
    P.coeffRef(i, clusterA[i]) = 1;
  }
  for (int i = 0; i < n_n; i++) {
    P.coeffRef(i+n_p, clusterB[i]) = 1;
  }
  
  SparseMatrix<double> A_ord = P * A_s * P.transpose();

  SparseMatrix<double> nonDiagAord = A_ord.topRightCorner(n_p, n_n);
  cout << nonDiagAord.nonZeros() << endl;

  SparseMatrix<double> nonDiagAs = A_s.topRightCorner(n_p, n_n);
  cout << nonDiagAs.nonZeros() << endl;
}