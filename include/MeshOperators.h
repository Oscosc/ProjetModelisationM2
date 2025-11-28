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
    const std::vector<int> getNeighbors(const Eigen::SparseMatrix<double>& L, const int vid);

    /**
     * @brief Compute a step of Laplacian diffusion function.
     * 
     * For each point of the mesh, it's new value is equal to the mean value of it's neighbors
     * at the previous step.
     * 
     * @param previousLaplacian Previous step value of the Laplacian
     * @param A Adjacency Matrix, used to determine neighbors
     * @return Value of the Laplacian at this new step
     */
    Eigen::VectorXd computeDiffuseLaplacianStep(
        const Eigen::VectorXd& previousLaplacian,
        const Eigen::SparseMatrix<double>& L,
        const unsigned int sourceID,
        const Eigen::VectorXi& b,
        const Eigen::VectorXd& bc
    );

    /**
     * @brief TODO
     * 
     * @param V 
     * @param F 
     * @param b 
     * @param in 
     * @param bc_values 
     * @return Eigen::VectorXd 
     */
    Eigen::VectorXd computeLinearSystemLaplacian(
        const Eigen::MatrixXd& V,
        const Eigen::SparseMatrix<double>& L,
        const Eigen::VectorXi& b,
        const Eigen::VectorXi& in,
        const Eigen::VectorXd& bcValues
    );

    /**
     * @brief TODO
     * 
     * @param V 
     * @param in 
     * @param source 
     * @param bOut 
     * @param bcOut 
     */
    void laplacianBoundaryValues(
        const Eigen::MatrixXd& V,
        const Eigen::VectorXi& in,
        const unsigned int source,
        Eigen::VectorXi& b,
        Eigen::VectorXd& bcValues
    );

    /**
     * @brief TODO
     * 
     * @param U 
     * @param V 
     * @param F 
     * @param L
     * @param in
     * @param b
     * @return * void 
     */
    void laplacianSmoothing(
        Eigen::MatrixXd& U,
        const Eigen::MatrixXd& V,
        const Eigen::MatrixXi& F,
        const Eigen::SparseMatrix<double>& L
    );

    namespace Transfert {

        inline auto identity = [](const Eigen::VectorXd& L_in) -> Eigen::VectorXd {
            return L_in;
        };

        inline auto sqrt = [](const Eigen::VectorXd& L_in) -> Eigen::VectorXd {
            return L_in.array().sqrt().matrix();
        };

        inline auto log = [](const Eigen::VectorXd& L_in) -> Eigen::VectorXd {
            return (L_in.array().abs() + 1.0).log() * L_in.array().sign();
        };

        inline auto sigmoid = [](const Eigen::VectorXd& L_in) -> Eigen::VectorXd {
            return (1.0 / (1.0 + (-L_in.array()).exp())).matrix();
        };

        inline auto windowSigmoid = [](const Eigen::VectorXd& L_in, double x_min = 0.0, double x_max = 0.4) -> Eigen::VectorXd 
        {
            const double k = 6.0; // Pente de la sigmoide

            double width = x_max - x_min;
            double k_norm = k / width;
            double center = (x_min + x_max) / 2.0;

            Eigen::ArrayXd x_scaled = (L_in.array() - center) * k_norm;
            return (1.0 / (1.0 + (-x_scaled).exp())).matrix();
        };

        inline std::function<Eigen::VectorXd(const Eigen::VectorXd&)> METHODS[] =
            {identity, sqrt, log, sigmoid, windowSigmoid};
    }

    void deformationLaplacian(
        Eigen::MatrixXd& V,
        const Eigen::Block<const Eigen::MatrixXd, 1, -1, false>& normal,
        const Eigen::VectorXd& laplacian,
        const double alpha,
        std::function<Eigen::VectorXd(const Eigen::VectorXd&)> transfertFunction = Transfert::windowSigmoid
    );
}