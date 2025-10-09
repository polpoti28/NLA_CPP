#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>

using std::cout;
using std::endl;
using Eigen::MatrixXd;
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

  cout << A_g.norm() << endl;

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


}

