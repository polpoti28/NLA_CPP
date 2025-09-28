#include <Eigen/Sparse>
using Eigen::SparseMatrix; using Eigen::Triplet;

struct StencilEntry { int dr, dc; double w; };

SparseMatrix<double> conv_to_mat(const Eigen::MatrixXd& H, int n, int m) {
    using Eigen::SparseMatrix;
    using Eigen::Triplet;

    const int Hx = H.rows();         // righe kernel
    const int Hy = H.cols();         // colonne kernel
    const int cr = Hx / 2;           // centro riga
    const int cc = Hy / 2;           // centro colonna

    std::vector<Triplet<double>> triplets;
    triplets.reserve(static_cast<size_t>(n) * m * Hx * Hy);

    // column-major flattening: q = row + col * n
    for (int row = 0; row < n; ++row) {
        for (int col = 0; col < m; ++col) {
            const int q = row + col * n;
            for (int k = 0; k < Hx; ++k) {
                for (int l = 0; l < Hy; ++l) {
                    const double w = H(k,l);      // H(riga, colonna)
                    if (w == 0.0) continue;
                    const int nr = row + (k - cr);
                    const int nc = col + (l - cc);
                    if ((unsigned)nr < (unsigned)n && (unsigned)nc < (unsigned)m) {
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
