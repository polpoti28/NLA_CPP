#include <Eigen/Dense>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

/*
* Modified GetVectorElt to be able to read
* LIS format with indices
*/
inline void GetVectorElt_fixed (const std::string& line, double& val) {
  double valR;
  int index;
  std::istringstream newline(line);
  newline >> index >> valR; 
  val = valR;
}


/*
* loadMarketVector function fixes:
* the function now uses GetVectorElt_fixed to 
* put in vec the values in the .mtx file.
* It basically skips the indices to just read
* the values
*/
template<typename VectorType>
bool loadMarketVector_fixed(VectorType& vec, const std::string& filename) {
  typedef typename VectorType::Scalar Scalar;
  std::ifstream in(filename.c_str(), std::ios::in);
  if(!in)
    return false;
  
  std::string line; 
  int n(0);
  do 
  { // Skip comments
    std::getline(in, line); eigen_assert(in.good());
  } while (line[0] == '%');
  std::istringstream newline(line);
  newline >> n;
  eigen_assert(n>0);
  vec.resize(n);
  int i = 0; 
  Scalar value; 
  while (std::getline(in, line) && (i < n)) {
    GetVectorElt_fixed(line, value); 
    vec(i++) = value; 
  }
  in.close();
  if (i!=n){
    std::cerr<< "Unable to read all elements from file " << filename << "\n";
    return false;
  }
  return true;
}