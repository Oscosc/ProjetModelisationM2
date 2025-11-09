#pragma once

#include <set>
#include <MeshOperators.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>

struct LaplacianArea {

    /** currently selected vertices indexes on the mesh */
    std::set<unsigned int> selected;

    /** vid of the source point in the selection area */
    unsigned int source;

    /** State of the laplacian computed */
    Eigen::VectorXd state;
};

/**
 * @brief Contain all information about a mesh, including original state, deformations
 * applied, current selected vertices, etc.
 * 
 * @author Oscar G, 2025
 */
class MeshObject
{
public:

    MeshObject(Eigen::MatrixXd& V, Eigen::MatrixXi& F);

    // FIXME : Tout les getters doivent devenir constant (modification de l'objet doivent se faire
    // via des fonction publiques - encapsulation)
    LaplacianArea& Laplacian() { return m_laplacian; }
    MeshOperators::AdjacencyMatrix& A() { return m_A; }
    Eigen::MatrixXd& V() { return m_V; }
    Eigen::MatrixXd& C() { return m_C; }
    Eigen::MatrixXi& F() { return m_F; }

private:

    /** Information of current selection for Laplacian computation */
    LaplacianArea m_laplacian;

    /** precomputed mesh's neighbors map */
    MeshOperators::AdjacencyMatrix m_A;

    /** matrix of mesh's vertices positions */
    Eigen::MatrixXd m_V;

    /** matrix of mesh's vertices colors */
    Eigen::MatrixXd m_C;

    /** matrix of mesh's faces, containing the 3 vertices that's form face triangle on each row */
    Eigen::MatrixXi m_F;
};