#include <MeshObject.h>

#include <igl/per_vertex_normals.h>
#include <igl/setdiff.h>

MeshObject::MeshObject(Eigen::MatrixXd& V, Eigen::MatrixXi& F) : m_V(V), m_baseV(V), m_F(F)
{
    m_C = Eigen::MatrixXd::Constant(m_V.rows(),3,1);
    igl::cotmatrix(m_V, m_F, m_L);
    igl::per_vertex_normals(m_V, m_F, m_N);
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

void MeshObject::selectionToLaplaceForm(Eigen::VectorXi& in, Eigen::VectorXi& b, Eigen::VectorXd& bc)
{
    // Suppression de la source dans la selection
    std::set<unsigned int> selectedNoSource = Laplacian().selected;
    selectedNoSource.erase(Laplacian().source);

    // Création du vecteur 'in' et copie des données
    in = Eigen::VectorXi(selectedNoSource.size());
    std::copy(selectedNoSource.begin(), selectedNoSource.end(), in.data());

    // Lancement de la résolution
    MeshOperators::laplacianBoundaryValues(V(), in, Laplacian().source, b, bc);
}

void MeshObject::initLaplacian()
{
    Eigen::VectorXd initialState = Eigen::VectorXd::Zero(this->L().cols());
    initialState[this->Laplacian().source] = 1.0;
    m_laplacian.state = initialState;
}

void MeshObject::stepLaplacian()
{
    Eigen::VectorXi in, b;
    Eigen::VectorXd bc;

    // Récupération des conditions aux limites
    selectionToLaplaceForm(in, b, bc);

    // Calcul de la diffusion
    m_laplacian.state = MeshOperators::computeDiffuseLaplacianStep(
        this->Laplacian().state,
        this->L(),
        this->Laplacian().source,
        b, bc
    );
}

void MeshObject::linearSolvingLaplacian()
{
    Eigen::VectorXi in, b;
    Eigen::VectorXd bc;

    selectionToLaplaceForm(in, b, bc);
    m_laplacian.state = MeshOperators::computeLinearSystemLaplacian(V(), L(), b, in, bc);
}

void MeshObject::setDeformationMethod(const int id)
{
    m_deformationMethod = MeshOperators::Transfert::METHODS[id];
}

void MeshObject::deformLaplacian(const double alpha)
{
    auto normal = N().row(Laplacian().source);
    MeshOperators::deformationLaplacian(m_V, normal, Laplacian().state, alpha, m_deformationMethod);
    // igl::per_vertex_normals(m_V, m_F, m_N); // Recalcul des normales
}

void MeshObject::laplacianSmoothing()
{
    MeshOperators::laplacianSmoothing(m_V, V(), F(), L());
}

void MeshObject::resetMesh()
{
    m_V = m_baseV;
}
