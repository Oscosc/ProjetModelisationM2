#include <MeshObject.h>

MeshObject::MeshObject(Eigen::MatrixXd& V, Eigen::MatrixXi& F) : m_V(V), m_F(F)
{
    m_C = Eigen::MatrixXd::Constant(m_V.rows(),3,1);
    m_A = MeshOperators::computeAdjacencyMatrix(m_F, m_V.rows());
}

void MeshObject::resetLaplacianArea()
{
    m_laplacian.selected.clear();
    m_C = Eigen::MatrixXd::Constant(this->V().rows(),3,1);
}

bool MeshObject::changeLaplacianSource(unsigned int newSourceVID)
{
    bool isSelected = this->Laplacian().selected.count(newSourceVID);

    if(isSelected) {
      m_C.row(m_laplacian.source) = SELECTION_COLOR;
      m_laplacian.source = newSourceVID;
      m_C.row(newSourceVID) = SOURCE_COLOR;
      return true;
    }
    return false;
}

bool MeshObject::addLaplacian(unsigned int vid)
{
    bool isSelected = this->Laplacian().selected.count(vid);

    if(!isSelected) {
        if(this->Laplacian().selected.empty()) {
            m_laplacian.source = vid;
            m_C.row(vid) = SOURCE_COLOR;
        } else {
            m_C.row(vid) = SELECTION_COLOR;
        }
        m_laplacian.selected.insert(vid);
        return true;
    }
    return false;
}

bool MeshObject::removeLaplacian(unsigned int vid)
{
    bool isSelected = this->Laplacian().selected.count(vid);

    if(isSelected) {
        m_C.row(vid) = BASE_COLOR;
        m_laplacian.selected.erase(vid);
        return true;
    }
    return false;
}

void MeshObject::initLaplacian()
{
    Eigen::VectorXd initialState = Eigen::VectorXd::Zero(this->A().cols());
    initialState[this->Laplacian().source] = 1.0;
    m_laplacian.state = initialState;
}

void MeshObject::stepLaplacian()
{
    m_laplacian.state = MeshOperators::computeDiffuseLaplacianStep(this->Laplacian().state, this->A(), this->Laplacian().source);
}

void MeshObject::linearSolvingLaplacian()
{
    Eigen::VectorXi b;
    Eigen::VectorXd bc;

    // Suppression de la source dans la selection
    std::set<unsigned int> selectedNoSource = Laplacian().selected;
    selectedNoSource.erase(Laplacian().source);

    // Création du vecteur 'in'
    Eigen::VectorXi in(selectedNoSource.size());

    // Copie dans un vecteur Eigen
    std::copy(selectedNoSource.begin(), selectedNoSource.end(), in.data());

    // Lancement de la résolution
    MeshOperators::laplacianBoundaryValues(V(), in, Laplacian().source, b, bc);
    m_laplacian.state = MeshOperators::computeLinearSystemLaplacian(V(), F(), b, in, bc);
}
