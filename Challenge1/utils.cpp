#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>

using namespace Eigen;
using namespace std;


SparseMatrix<double> conv_to_mat(const MatrixXd& H, int n, int m) {
  SparseMatrix<double> A1(n*m, n*m);
  int H_x = H.rows();
  int H_y = H.cols();

  for (int i = 0; i < n; i++) {
    for (int j = 0; j < m; j++) {
      for (int l = 0; l < H_y; l++) {
        for (int k = 0; k < H_x; k++) {
         if ((i >= (H_y - 1)/2 - l) && (j >= (H_x - 1)/2 - k) 
                  && (i <= (H_y - 1)/2 - l + n*m) && (j <= (H_x - 1)/2 - k + n*m))  {
          A1.coeffRef(i - (H_y - 1)/2 + l, j - (H_x - 1)/2 + k) = H(l,k);
          }
        }
      }
    }
  }

  return A1;
}
