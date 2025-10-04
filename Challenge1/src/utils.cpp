#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <iostream>
#include <string>
#include "stb_image.h"
#include "stb_image_write.h"

using Eigen::StorageOptions::RowMajor;

/* Converts a 2D convolution kernel into a sparse 
 * matrix representing the linear operator of the
 * convolution on the image, wich is represented
 * as a vector */
Eigen::SparseMatrix<double> conv_to_mat(const Eigen::MatrixXd& H, int n, int m) {
    typedef Eigen::Triplet<double> T;
    const int Hx = H.rows();         
    const int Hy = H.cols();         
    const int cr = Hx / 2;          // Center row and center column
    const int cc = Hy / 2;          // of the kernel (H)

    std::vector<T> triplets;
    triplets.reserve(static_cast<size_t>(n) * m * Hx * Hy);
    for (int row = 0; row < n; row++) {
        for (int col = 0; col < m; col++) {
            const int q = row + col * n; // q is the row index of A
            for (int k = 0; k < Hx; k++) {
                for (int l = 0; l < Hy; l++) {
                    const double w = H(k,l);
                    if (w == 0.0) continue;
                    const int nr = row + (k - cr); // This allows to calculate the 
                    const int nc = col + (l - cc); // contribution of the original image pixel
                    if (nr >= 0 && nr < n && nc >= 0 && nc < m) {
                        const int q2 = nr + nc * n;  // q2 is the column index of A
                        triplets.emplace_back(q, q2, w);
                    }
                }
            }
        }
    }

    Eigen::SparseMatrix<double> A(n*m, n*m);
    A.setFromTriplets(triplets.begin(), triplets.end());
    A.makeCompressed();
    return A;
}

/* This function takes an image saved as
*  a matrix of doubles and converts it to
*  unsigned char to then save it as a .png. */
int saveImg(const std::string output_string, Eigen::MatrixXd img_d,
            const std::string type, int height, int width) {
  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic, RowMajor> img(height, width);
  img = img_d.unaryExpr([](double val) -> unsigned char {
    return static_cast<unsigned char>(val);
  });

  // Save the image using stb_image_write
  const std::string path  = output_string;
  if (stbi_write_png(path.c_str(), width, height, 1,
                     img.data(), width) == 0) {
    std::cerr << "Error: Could not save "<< type << "image" << std::endl;
    return 1;
  }
  return 0;
}

/*
* saveMarketVector is deprecated, we implement
* a custom function that saves a vector with
* vector coordinate real general format.
* This allows LIS and Eigen formats to communicate correctly. */
void saveMarketVectorCRL(const char* filename, const Eigen::VectorXd v) {
    const long n = v.size();
    FILE* fp = fopen(filename, "w");
    fprintf(fp, "%%%%MatrixMarket vector coordinate real general\n");
    fprintf(fp, "%ld 1\n", n);
    for (int i = 0; i < n; i++) {
        fprintf(fp, "%d %f\n", i+1, v(i));
    }
    fclose(fp);
}