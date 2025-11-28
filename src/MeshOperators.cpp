#include <MeshOperators.h>

#include <igl/cotmatrix.h>
#include <igl/slice_into.h>
#include <igl/setdiff.h>

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

const std::vector<int> MeshOperators::getNeighbors(const Eigen::SparseMatrix<double>& L, const int vid)
{
    std::vector<int> neighbors;
    const Eigen::SparseMatrix<double>& L_T = L; 
    
    for (Eigen::SparseMatrix<double>::InnerIterator it(L_T, vid); it; ++it)
    {
        int j = it.row(); 
        if (j != vid)
            neighbors.push_back(j);
    }
    return neighbors;
}

Eigen::VectorXd MeshOperators::computeDiffuseLaplacianStep(const Eigen::VectorXd& previousLaplacian,
    const Eigen::SparseMatrix<double>& L, const unsigned int sourceID, const Eigen::VectorXi& b, const Eigen::VectorXd& bc)
{
    const double DELTA_T = 5e-2;

    // l(t) = l(t-1) + delta * l(t-1) * L
    Eigen::VectorXd gradient = L * previousLaplacian;
    Eigen::VectorXd rawResult = previousLaplacian + DELTA_T * gradient; 

    // Conditions aux limites
    for(unsigned int i = 0; i < b.size(); ++i) {
        assert(previousLaplacian[b[i]] == bc[i]); // Pas de changement sur la condition
        rawResult[b[i]] = bc[i];
    }
    
    return rawResult;
}

Eigen::VectorXd MeshOperators::computeLinearSystemLaplacian(const Eigen::MatrixXd& V, const Eigen::SparseMatrix<double>& L,
    const Eigen::VectorXi& b, const Eigen::VectorXi& in, const Eigen::VectorXd& bcValues)
{
    // On ne veut conserver que L_in_in et L_in_b (cf. Documentation), donc on slice la matrice pour ne conserver que
    // ces valeurs
    Eigen::SparseMatrix<double> L_in_in, L_in_b;
    igl::slice(L, in, in, L_in_in);
    igl::slice(L, in, b,  L_in_b);

    // Objectif de la résolution L_in_in * in_values = - L_in_b * bc_values, avec in_values = inconnues
    // Calcul de RHS (Right hand side)
    Eigen::VectorXd RHS = L_in_b * bcValues;

    // Phase de résolution du système
    Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver(-L_in_in);
    if(solver.info() != Eigen::Success) {
        std::cerr << "Echec de la factorisation du système" << std::endl;
        exit(1);
    }
    
    // Calcul de la solution si la factorisation a fonctionné
    Eigen::VectorXd u_in = solver.solve(RHS);

    // Ré-assemblage des différentes parties pour avoir l'info de valeur de tout les sommets
    Eigen::VectorXd Z = Eigen::VectorXd::Zero(V.rows());
    igl::slice_into(bcValues, b, Z);
    igl::slice_into(u_in, in, Z);

    return Z;
}

void MeshOperators::laplacianBoundaryValues(const Eigen::MatrixXd &V, const Eigen::VectorXi &in,
    const unsigned int source, Eigen::VectorXi &b, Eigen::VectorXd &bcValues)
{
    // Creation d'un vecteur d'indices
    Eigen::VectorXi all = Eigen::VectorXi::LinSpaced(V.rows(), 0, V.rows() - 1);
    Eigen::VectorXi IA;

    // le vecteur d'indices b est par définition : all - in
    igl::setdiff(all, in, b, IA);

    // Toutes les conditions de bord sont des 0...
    bcValues = Eigen::VectorXd::Zero(b.size());

    // ...sauf la source qui est à 1
    int sourceIndex;
    Eigen::Array<bool, Eigen::Dynamic, 1> mask = (b.array() == (int)source); // Masque de recherche
    bool dummy = mask.maxCoeff(&sourceIndex); // On assume qu'il n'existe que chaque élément du vecteur est unique
    
    bcValues[sourceIndex] = 1;
}

void MeshOperators::deformationLaplacian(Eigen::MatrixXd &V, const Eigen::Block<const Eigen::MatrixXd, 1, -1, false>& normal,
    const Eigen::VectorXd &laplacian, const double alpha, std::function<Eigen::VectorXd(const Eigen::VectorXd&)> transfertFunction)
{
    const Eigen::VectorXd transferedLaplacian = transfertFunction(laplacian);
    V += (transferedLaplacian * normal) * alpha;
}
