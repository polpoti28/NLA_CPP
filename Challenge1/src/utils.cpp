#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cctype>
#include "stb_image.h"
#include "stb_image_write.h"

using namespace Eigen;
using namespace std;

/* Converts a 2D convolution kernel into a sparse 
 * matrix representing the linear operator of the
 * convolution on the image, wich is represented
 * as a vector */

SparseMatrix<double> conv_to_mat(const MatrixXd& H, int n, int m) {
    using Eigen::SparseMatrix;
    using Eigen::Triplet;

    const int Hx = H.rows();         
    const int Hy = H.cols();         
    const int cr = Hx / 2;           
    const int cc = Hy / 2;

    vector<Triplet<double>> triplets;
    triplets.reserve(static_cast<size_t>(n) * m * Hx * Hy);

    for (int row = 0; row < n; ++row) {
        for (int col = 0; col < m; ++col) {
            const int q = row + col * n;
            for (int k = 0; k < Hx; ++k) {
                for (int l = 0; l < Hy; ++l) {
                    const double w = H(k,l);
                    if (w == 0.0) continue;
                    const int nr = row + (k - cr);
                    const int nc = col + (l - cc);
                    if (nr >= 0 && nr < n && nc >= 0 && nc < m) {
                        const int q2 = nr + nc * n;  // stesso indexing column-major
                        triplets.emplace_back(q, q2, w);
                    }
                }
            }
        }
    }

    SparseMatrix<double> A(n*m, n*m);
    A.setFromTriplets(triplets.begin(), triplets.end());
    A.makeCompressed();
    return A;
}

/* This function takes an image saved as
*  a matrix of doubles and converts it to
*  unsigned char to then save it as a .png. */

int saveImg(const string output_string, MatrixXd img_d,
            const string type, int height, int width) {
  Matrix<unsigned char, Dynamic, Dynamic, RowMajor> img(height, width);
  img = img_d.unaryExpr([](double val) -> unsigned char {
    return static_cast<unsigned char>(val);
  });

  // Save the image using stb_image_write
  const string path  = output_string;
  if (stbi_write_png(path.c_str(), width, height, 1,
                     img.data(), width) == 0) {
    cerr << "Error: Could not save "<< type << "image" << endl;
    return 1;
  }
  return 0;
}

/*
* saveMarketVector is deprecated, we implement
* a custom function that saves a vector with
* vector coordinate real general format
*/
void saveMarketVectorCRL(const char* filename, const VectorXd v) {
    const long n = v.size();
    FILE* fp = fopen(filename, "w");
    fprintf(fp, "%%%%MatrixMarket vector coordinate real general\n");
    fprintf(fp, "%ld 1\n", n);
    for (int i = 0; i < n; i++) {
        fprintf(fp, "%d %f\n", i+1, v(i));
    }
    fclose(fp);
}