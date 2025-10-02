#include <Eigen/Dense>
#include <iostream>
#include <Eigen/Sparse>
#include <unsupported/Eigen/SparseExtra>
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
  Eigen::Map<const VectorXd> w(noisyImg.data(), noisyImg.size());

  // 2) Verifica dimensioni
  assert(v.size() == height * width);
  assert(w.size() == height * width);

  // Calculate the euclidean norm
  double v_norm = v.norm(); 
  cout<<"Euclidean norm of v:"<<v_norm<<endl;

  saveImg("noisyImg.png", noisyImg, "noisy", height, width);

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
  blurv = blurv.cwiseMax(0.0).cwiseMin(255.0);
  Eigen::Map<const MatrixXd> blurImg(blurv.data(), height, width);
  saveImg("blurImg.png", blurImg, "blurred", height, width);
  
  MatrixXd H_sh1(3,3);
  H_sh1 << 0, -2, 0,
           -2, 9, -2,
           0, -2, 0;
  
  SparseMatrix<double> A2 = conv_to_mat(H_sh1, height, width);
  cout << height << " " << width << endl;
  cout << A2.nonZeros() << endl;

  SparseMatrix<double> A2_sp = SparseMatrix<double>(A2.transpose()) - A2;
  cout << "Norm of skew-symmetric part of A2: " << A2_sp.norm() << endl;

  VectorXd sharpv = A2 * v;
  sharpv = sharpv.cwiseMax(0.0).cwiseMin(255.0);
  Map<const MatrixXd> sharpImg(sharpv.data(), height, width);
  saveImg("sharpImg.png", sharpImg, "sharpened", height, width);


  ifstream file("sol.mtx");
  if (!file) {
    // Exporting A2 and v in Matrix Market format
    saveMarket(A2, "A2.mtx");
    saveMarketVectorCRL("w.mtx", w);
  }

  VectorXd x;
  if (!loadMarketVector_fixed(x, "sol.mtx")) {
    std::cerr << "Failed to load sol.mtx" << std::endl;
    return 1;
  }
  x = x.cwiseMax(0.0).cwiseMin(255.0);
  cout << x.size() << endl;
  
  Map<const MatrixXd> xImg(x.data(), height, width);
  saveImg("x.png", xImg, "sol1", height, width);

  MatrixXd H_ed2(3,3);
  H_ed2 << -1, -2, -1,
            0, 0, 0,
            1, 2, 1;

  SparseMatrix<double> A3 = conv_to_mat(H_ed2, height, width);
  SparseMatrix<double> A3_sp = SparseMatrix<double>(A3.transpose()) - A3;
  cout << "Norm of skew-symmetric part of A3: " << A3_sp.norm() << endl;

  VectorXd vEdge = A3 * v;
  vEdge = vEdge.cwiseMax(0.0).cwiseMin(255.0);
  Map<const MatrixXd> vEdgeImg(vEdge.data(), height, width);
  saveImg("edgeImg.png", vEdgeImg, "edge", height, width);

  const int nm = height * width;
  //DiagonalMatrix<double,nm>::Identity	mat1(nm, nm);

  SparseMatrix<double> I(nm, nm);
  SparseMatrix<double> A3_I(nm, nm);
  I.setIdentity();
  I  = 3 * I;

  A3_I  = A3 + I;

  cout << I.size() << endl;
  DiagonalPreconditioner<double> D(A3_I);
  BiCGSTAB<SparseMatrix<double> > solver;
 // solver.setMaxIterations(1000);
  solver.setTolerance(1e-8);
  solver.compute(A3_I);
  VectorXd y; 
  y = solver.solve(w);
  std::cout << "#iterations:     " << solver.iterations() << std::endl;
  std::cout << "estimated error: " << solver.error() << std::endl; 



  y = y.cwiseMax(0.0).cwiseMin(255.0);
  Map<const MatrixXd> yImg(y.data(), height, width);
  saveImg("y.png", yImg, "y", height, width);


int ret = system("./lis_bash_script.sh A3.mtx 1 sol_tmp.mtx OUTPUT_FILE.mtx HISTORY.txt");

if (ret != 0) {
    std::cerr << "Failed to run the bash script!" << std::endl;
}
  



  return 0;








}
