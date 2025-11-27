#pragma once

#include <set>
#include <MeshOperators.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#define SOURCE_COLOR    Eigen::RowVector3d(1, 0, 0)
#define SELECTION_COLOR Eigen::RowVector3d(1, 1, 0)
#define BASE_COLOR      Eigen::RowVector3d(1, 1, 1)

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

    /************************************ GETTERS ************************************/

    const LaplacianArea& Laplacian() { return m_laplacian; }
    const MeshOperators::AdjacencyMatrix& A() { return m_A; }
    const Eigen::MatrixXd& V() { return m_V; }
    const Eigen::MatrixXd& C() { return m_C; }
    const Eigen::MatrixXi& F() { return m_F; }

    /*********************************************************************************/

    /**
     * @brief Reset the laplacian area for this object, including color of vertices
     * 
     */
    void resetLaplacianArea();

    /**
     * @brief Change the laplacian source point with new vid if vid already is in selected area
     * 
     * @param newSourceVID vid of the new source point
     * @return true if modified, false otherwise
     */
    bool changeLaplacianSource(unsigned int newSourceVID);

    /**
     * @brief Add a vertex to the laplacian selection area
     * 
     * @param vid vid of the new source point
     * @return true if correctly added, false otherwise
     */
    bool addLaplacian(unsigned int vid);

    /**
     * @brief Remove a vertex from the laplacian selection area
     * 
     * @param vid vid of the removed point
     * @return true if correctly removed, false otherwise
     */
    bool removeLaplacian(unsigned int vid);

    void initLaplacian();

    void stepLaplacian();

    void linearSolvingLaplacian();

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