#include <MeshOperators.h>

MeshOperators::AdjacencyMatrix MeshOperators::computeAdjacencyMatrix(const Eigen::MatrixXi &F, const int sizeV)
{
    // Constructing a sparse adjency matrix
    AdjacencyMatrix A(sizeV, sizeV);

    std::vector<Eigen::Triplet<double>> triplets;
    for (int i = 0; i < F.rows(); ++i)
    {
        int v0 = F(i,0), v1 = F(i,1), v2 = F(i,2);
        
        // Links
        triplets.emplace_back(v0, v1, 1.0);
        triplets.emplace_back(v1, v2, 1.0);
        triplets.emplace_back(v0, v2, 1.0);

        // Complementary of elements before
        triplets.emplace_back(v1, v0, 1.0);
        triplets.emplace_back(v2, v1, 1.0);
        triplets.emplace_back(v2, v0, 1.0);
    }
    A.setFromTriplets(triplets.begin(), triplets.end());

    return A;
}

const std::vector<int> MeshOperators::getNeighbors(const AdjacencyMatrix& A, const int vid)
{
    std::vector<int> neighbors;
    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A, vid); it; ++it)
    {
        int j = it.col();
        if (it.value() != 0.0 && j != vid)
            neighbors.push_back(j);
    }
    return neighbors;
}

Eigen::VectorXd MeshOperators::computeDiffuseLaplacianStep(const Eigen::VectorXd& previousLaplacian, const AdjacencyMatrix& A)
{
    // L(t) = (A * L(t-1)) / (A * [1])
    return (A * previousLaplacian).cwiseQuotient(A * Eigen::VectorXd::Ones(A.cols()));
}
