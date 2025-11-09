#pragma once

#include <iostream>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace MeshOperators
{
    /**
     * @brief Adjacency matrix represents links between vertices in the current mesh.
     * Using this implem. enable matrix operations for laplacian computation and other functions
     */
    using AdjacencyMatrix = Eigen::SparseMatrix<double, Eigen::RowMajor>;

    /**
     * @brief Compute an adjacency matrix for each vertex of the mesh, using informations
     * in faces matrix loaded.
     * 
     * @param F Faces matrix obtained while loading mesh
     * @param sizeV Number of vertices in the mesh (eq. to V.rows())
     * @return Adjacency matrix such as A(i,j) == 1 if vertex i is adjacent to vertex j,
     * 0 otherwise
     */
    AdjacencyMatrix computeAdjacencyMatrix(const Eigen::MatrixXi& F, const int sizeV);

    /**
     * @brief Wrapper for returning a list of neighbors vertex IDs from an AdjacencyMatrix.
     * 
     * @param A Adjacency Matrix
     * @param vid Vertex ID for wich we need to find the neighbors
     * @return A list of vertex IDs representing neighbors
     */
    const std::vector<int> getNeighbors(const AdjacencyMatrix& A, const int vid);

    Eigen::VectorXd computeDiffuseLaplacianStep(const Eigen::VectorXd& previousLaplacian, const AdjacencyMatrix& A);
}