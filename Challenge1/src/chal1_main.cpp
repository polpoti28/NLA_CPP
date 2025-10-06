#include <Eigen/Dense>
#include <iostream>
#include <Eigen/Sparse>
#include <unsupported/Eigen/SparseExtra>
#include <string>
#include "utils.h"
#include <time.h>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

using Eigen::MatrixXd;
using Eigen::SparseMatrix;
using Eigen::VectorXd;
using std::cout;
using std::cerr;
using std::endl;

int main(int argc, char* argv[]) {
  if (argc < 2 ) {
    cerr << "Usage: " << argv[0] << " <image_path>" << endl;
    return 1;
  }

  const char* input_image_path = argv[1];

  // ---------
  // TASK 1
  // ---------

  // Loading the image using stb_image
  int width, height, channels;
  unsigned char* image_data = stbi_load(input_image_path, &width, &height, &channels, 1);
  if (!image_data) {
    std::cerr << "Error: Could not load image " << input_image_path << std::endl;
    return 1;
  }

  std::cout << "Image loaded: " << width << "x" << height << " with " << channels << " channels." << std::endl;
  
  MatrixXd matImg(height, width);
  // Filling the matrix with image data
  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) { 
      int index = i*width + j;
      matImg(i, j) = static_cast<double>(image_data[index]);
    }
  }

  cout << "Size of the matrix: " << matImg.rows() << " " << matImg.cols() << endl;

  stbi_image_free(image_data);
  
  // ---------
  // TASK 2
  // ---------

  // We apply random noise by using Eigen MatrixXd::Random
  // It generates a matrix with random values between [-1,1]
  MatrixXd noise = MatrixXd::Random(height, width) * 40;
  MatrixXd noisyImg = matImg + noise;
  noisyImg = noisyImg.cwiseMax(0.0).cwiseMin(255.0);  
  saveImg("../images/noisyImg.png", noisyImg, "noisy", height, width);

  // ---------
  // TASK 3
  // ---------
  
  Eigen::Map<const VectorXd> v(matImg.data(), matImg.size()); 
  Eigen::Map<const VectorXd> w(noisyImg.data(), noisyImg.size());

  // Verify the dimensions
  assert(v.size() == height * width);
  assert(w.size() == height * width);

  // Calculate the euclidean norm
  double v_norm = v.norm(); 
  cout << "Euclidean norm of v: " << v_norm << endl;

  // ---------
  // TASK 4
  // ---------

  MatrixXd H_av1(3,3);
  H_av1 << 1, 1, 0,
            1, 2, 1,
            0, 1, 1;
  H_av1 = H_av1 / 8.0;
  
  // Constructing A1
  SparseMatrix<double> A1 = conv_to_mat(H_av1, height, width);
  
  
  cout << "Number of nonzero entries of A1: " << A1.nonZeros() << endl;
  
  // ---------
  // TASK 5
  // ---------

  /*
  * We apply the filter and reconstruct the matrix 
  * for the image */
  VectorXd blurv = A1 * w;
  blurv = blurv.cwiseMax(0.0).cwiseMin(255.0);
  Eigen::Map<const MatrixXd> blurImg(blurv.data(), height, width);
  saveImg("../images/blurImg.png", blurImg, "blurred", height, width);
  
  // ---------
  // TASK 6
  // ---------

  MatrixXd H_sh1(3,3);
  H_sh1 << 0, -2, 0,
           -2, 9, -2,
           0, -2, 0;
  
  SparseMatrix<double> A2 = conv_to_mat(H_sh1, height, width);
  SparseMatrix<double> A2_sp = SparseMatrix<double>(A2.transpose()) - A2;
  cout << "Norm of skew-symmetric part of A2: " << A2_sp.norm();
  if (A2_sp.norm() < 1e-16) {
    cout << " --> A2 is symmetric" << endl;
  } else {
    cout << " --> A2 is not symmetric" << endl;
  }
  cout << "Number of nonzero entries of A2: " << A2.nonZeros() << endl;

  /* To check if A2 is positive definite we apply
  * Cholesky decomposition */

  Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> chol(A2);
  if (chol.info() == Eigen::Success) {
    cout << "A2 is positive definite" << endl;
  } else {
    cout << "A2 is not positive definite" << endl;
  }

  // ---------
  // TASK 7
  // ---------
  
  VectorXd sharpv = A2 * v;
  sharpv = sharpv.cwiseMax(0.0).cwiseMin(255.0);
  Eigen::Map<const MatrixXd> sharpImg(sharpv.data(), height, width);
  saveImg("../images/sharpImg.png", sharpImg, "sharpened", height, width);

  // ---------
  // TASK 8
  // ---------

  std::ifstream file("sol.mtx");
  if (!file) {
    // Exporting A2 and v in Matrix Market format
    saveMarket(A2, "../data/A2.mtx");
    saveMarketVectorCRL("../data/w.mtx", w);
  }

  cout << endl << "---------- LIS OUTPUT ----------" << endl;
  
  int ret = system("../data/run_test.sh");

  if (ret != 0) {
    std::cerr << "Failed to run the bash script!" << std::endl;
  }

  cout << "--------------------------------" << endl << endl;

  VectorXd x;
  if (!loadMarketVector_fixed(x, "../data/sol.mtx")) {
    std::cerr << "Failed to load sol.mtx" << std::endl;
    return 1;
  }
  x = x.cwiseMax(0.0).cwiseMin(255.0);
  
  // ---------
  // TASK 9
  // ---------

  Eigen::Map<const MatrixXd> xImg(x.data(), height, width);
  saveImg("../images/x.png", xImg, "sol1", height, width);

  // ---------
  // TASK 10
  // ---------

  MatrixXd H_ed2(3,3);
  H_ed2 << -1, -2, -1,
            0, 0, 0,
            1, 2, 1;

  SparseMatrix<double> A3 = conv_to_mat(H_ed2, height, width);
  SparseMatrix<double> A3_sp = SparseMatrix<double>(A3.transpose()) - A3;
  cout << "Number of non zero entries of A3: " << A3.nonZeros() << endl; 
  cout << "Norm of skew-symmetric part of A3: " << A3_sp.norm();
  if (A3_sp.norm() < 1e-16) {
    cout << " --> A3 is symmetric" << endl;
  } else {
    cout << " --> A3 is not symmetric " << endl;
  }

  // ---------
  // TASK 11
  // ---------

  VectorXd vEdge = A3 * v;
  vEdge = vEdge.cwiseMax(0.0).cwiseMin(255.0);
  Eigen::Map<const MatrixXd> vEdgeImg(vEdge.data(), height, width);
  saveImg("../images/edgeImg.png", vEdgeImg, "edge", height, width);

  // ---------
  // TASK 12
  // ---------

  const int nm = height * width;

  SparseMatrix<double> I(nm, nm);
  SparseMatrix<double> A3_I(nm, nm);
  I.setIdentity();
  I  = 3 * I;
  A3_I  = A3 + I;

  clock_t start = clock();
  Eigen::BiCGSTAB<SparseMatrix<double>> solver;
  solver.setTolerance(1e-8);
  solver.compute(A3_I);
  VectorXd y; 
  y = solver.solve(w);
  clock_t end = clock();

  cout << "Elapsed time: " << (end - start) / CLOCKS_PER_SEC << "s" << endl;

  double final_res = ((w - A3_I*y).norm()) / (w.norm());

  std::cout << "#iterations:     " << solver.iterations() << std::endl;
  std::cout << "estimated error: " << solver.error() << std::endl; 
  std::cout << "Final residual:  " << final_res << std::endl;

  // ---------
  // TASK 13
  // ---------

  y = y.cwiseMax(0.0).cwiseMin(255.0);
  Eigen::Map<const MatrixXd> yImg(y.data(), height, width);
  saveImg("../images/y.png", yImg, "y", height, width);

  return 0;
}