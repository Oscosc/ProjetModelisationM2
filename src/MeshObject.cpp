#include <MeshObject.h>

MeshObject::MeshObject(Eigen::MatrixXd& V, Eigen::MatrixXi& F) : m_V(V), m_F(F)
{
    m_C = Eigen::MatrixXd::Constant(m_V.rows(),3,1);
    m_A = MeshOperators::computeAdjacencyMatrix(m_F, m_V.rows());
}
