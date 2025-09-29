#include <Eigen/Dense>
#include <iostream>
#include <Eigen/Sparse>
#include <string>
#include "utils.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

using namespace Eigen;
using namespace std;

int main(int argc, char* argv[]) {
  if (argc < 2 ) {
    cerr << "Usage: " << argv[0] << " <image_path>" << endl;
    return 1;
  }

  const char* input_image_path = argv[1];

  // Load the image using stb_image
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

  cout << matImg.size() << endl;

  stbi_image_free(image_data);
  
  // We apply random noise by using Eigen MatrixXd::Random
  // It generates a matrix with random values between [-1,1]
  MatrixXd noise = MatrixXd::Random(height, width) * 40;
  MatrixXd noisyImg = matImg + noise;
  noisyImg = noisyImg.cwiseMax(0.0).cwiseMin(255.0);  

  Eigen::Map<const VectorXd> v(matImg.data(), matImg.size()); // size() == m*n
  Eigen::Map<const VectorXd> w(noisyImg.data(),    noisyImg.size());

  // 2) Verifica dimensioni
  assert(v.size() == height * width);
  assert(w.size() == height * width);

  // 3) Norma euclidea di v
  double v_norm = v.norm(); 
  cout<<"norma euclidea di v:"<<v_norm<<endl;


  saveImg("noisyImg.png", noisyImg, "noisy", height, width);

  /*
  Matrix<unsigned char, Dynamic, Dynamic, RowMajor> img(height, width);
  img = noisyImg.unaryExpr([](double val) -> unsigned char {
    return static_cast<unsigned char>(val);
  });

  // Save the image using stb_image_write
  const string output_image_path1 = "noisyImg.png";
  if (stbi_write_png(output_image_path1.c_str(), width, height, 1,
                     img.data(), width) == 0) {
    cerr << "Error: Could not save noisy image" << endl;

    return 1;
  }
  */
  
  MatrixXd H_av1(3,3);
  H_av1 << 1, 1, 0,
            1, 2, 1,
            0, 1, 1;
  H_av1 = H_av1 / 8.0;
  
  // Constructing A1
  SparseMatrix<double> A1 = conv_to_mat(H_av1, height, width);
  
  cout << height << " " << width << endl;
  cout << A1.nonZeros() << endl;
  
  /*
  * We apply the filter and reconstruct the matrix 
  * for the image */
  VectorXd blurv = A1 * w;
  Eigen::Map<const MatrixXd> blurImg(blurv.data(), height, width);
  
  saveImg("blurImg.png", blurImg, "blurred", height, width);
  
  return 0;
}
