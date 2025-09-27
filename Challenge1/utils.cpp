#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>

using namespace Eigen;
using namespace std;


SparseMatrix<double> conv_to_mat(const MatrixXd& H, int n, int m) {
  SparseMatrix<double> A1(n*m, n*m);
  int H_x = H.cols();
  int H_y = H.rows();

  typedef Triplet<double> T;
  vector<T> tripletList;
  
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < m; j++) {
      for (int l = 0; l < H_y; l++) {
        for (int k = 0; k < H_x; k++) {
          int row = i * m + j;
          int col = (i - ((H_y - 1) / 2 + l)) * m 
                     + (j - ((H_x - 1) / 2 + k));
          if (row>=0 && col >= 0 && row<n*m && col<n*m)
          {
            tripletList.push_back(T(row, col, H(l,k)));
          }
        }
      }
    }
  }
  
  A1.setFromTriplets(tripletList.begin(), tripletList.end());

  return A1;
}
