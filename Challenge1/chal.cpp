#include <Eigen/Dense>
#include <iostream>

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
      int index = (i*width + j)*channels;
      matImg(i, j) = static_cast<double>(image_data[index]);
    }
  }

  cout << matImg << endl;
  cout << matImg.size() << endl;

  stbi_image_free(image_data);
  
  // We apply random noise by using Eigen MatrixXd::Random
  // It generates a matrix with random values between [-1,1]
  MatrixXd noise = MatrixXd::Random(height, width) * 40;
  MatrixXd noisyImg = matImg + noise;
  noisyImg = noisyImg.cwiseMax(0.0).cwiseMin(255.0);  

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

  return 0;
}
