#include <Eigen/Dense>
#include <string>

using namespace Eigen;
using namespace std;

SparseMatrix<double> conv_to_mat(const MatrixXd& H, int n, int m);
int saveImg(const string output_string, MatrixXd img_d, const string type, int height, int width);
