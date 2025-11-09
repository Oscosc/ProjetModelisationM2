#pragma once

#include <set>
#include <map>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <Eigen/Sparse>
#include <MeshOperators.h>

#define SOURCE_COLOR    Eigen::RowVector3d(1, 0, 0)
#define SELECTION_COLOR Eigen::RowVector3d(1, 1, 0)
#define BASE_COLOR      Eigen::RowVector3d(1, 1, 1)

/**
 * @brief Contain UI interface and mesh loader system.
 * Only one mesh can be loaded at once.
 * 
 * @author Oscar G, 2025
 */
class ModelingApp
{

/****************************** METHODS ******************************/
public:

    /**
     * @brief Construct a new Modeling App with a ImGui Menu by default
     * 
     * @param V vertices matrix
     * @param F faces matrix
     * @param withMenu enable ImGui menu
     */
    ModelingApp(Eigen::MatrixXd& V, Eigen::MatrixXi& F, const bool withMenu = true);

    /**
     * @brief Launch the current application
     * 
     */
    void launch();

    /**
     * @brief Getter for the viewer opbject wrapped in this modeling application
     * 
     * @return igl::opengl::glfw::Viewer& viewer attribute
     */
    igl::opengl::glfw::Viewer& viewer();

private:

    /**
     * @brief Add mouse callbacks to this application, making it capable of
     * retrieving a user selected vertice when clicking
     * 
     */
    void addMouseCallback();

    /**
     * @brief Add ImGui menu to this application viewer
     * 
     */
    void addMenu();

    /**
     * @brief Update color of a vertex based on 'vid' argument.
     * When 'canDeactivate' is false, selecting an already selected vertex will have no effect
     * on the selection. Otherwise, it will remove the vertex from the current selection
     * 
     * @param vid vertex id to add/remove to current selection
     * @param canDeactivate true by default, enabling to remove item from selection.
     */
    void updateVertexColor(const unsigned int vid, const bool canDeactivate = true);

    /**
     * @brief Find the n-topological ring of a point
     * 
     * @param vid vertex id (source)
     * @param ringLevel level of topological ring to find (1 is neighbors linked with an edge to
     * the source vertex)
     * @param fill if true, il will return n-topological ring and 1 -> n-1 rings, otherwise it
     * will return only the selected ring
     * @return set of vertices of the topological ring
     */
    std::set<unsigned int> getTopologicalRing(const unsigned int vid, const unsigned int ringLevel);

    void setColorBasedOnLaplacian();

/***************************** ATTRIBUTES ****************************/
private:

    /** viewer object, used to display current mesh */
    igl::opengl::glfw::Viewer m_viewer;

    /** plugin object, used to link the menu to viewer */
    igl::opengl::glfw::imgui::ImGuiPlugin m_plugin;

    /** menu object, used to display various tools to perform functions on the mesh */
    igl::opengl::glfw::imgui::ImGuiMenu m_menu;

    /** matrix of mesh's vertices positions */
    Eigen::MatrixXd m_V;

    /** matrix of mesh's vertices colors */
    Eigen::MatrixXd m_C;

    /** matrix of mesh's faces, containing the 3 vertices that's form face triangle on each row */
    Eigen::MatrixXi m_F;

    /** default mesh color. DEPRECATED */
    Eigen::RowVector4f m_meshColor = Eigen::RowVector4f(0.8f, 0.8f, 0.8f, 1.0f);

    /** currently selected vertices indexes on the mesh */
    std::set<unsigned int> m_selectedIndexes;

    /** precomputed mesh's neighbors map */
    MeshOperators::AdjacencyMatrix m_A;

    /** INTERFACE : define if the next clicked point will set the source */
    bool m_nextIsSource = false;

    /** vid of the source point in the selection area */
    unsigned int m_source;

    Eigen::VectorXd m_laplacianState;
};