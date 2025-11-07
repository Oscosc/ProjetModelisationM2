#pragma once

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>

#include <set>
#include <map>

#define SOURCE_COLOR    Eigen::RowVector3d(1, 0, 0)
#define SELECTION_COLOR Eigen::RowVector3d(1, 1, 0)
#define BASE_COLOR      Eigen::RowVector3d(1, 1, 1)

class ModelingApp : public igl::opengl::glfw::Viewer
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
    void addMouseCallback();
    void addMenu();
    void updateVertexColor(const unsigned int vid, const bool canDeactivate = true);

    std::vector<std::set<int>> getNeighborsMap(const Eigen::MatrixXi& F);

    std::set<unsigned int> getTopologicalRing(const unsigned int ringLevel);

/***************************** ATTRIBUTES ****************************/
private:
    igl::opengl::glfw::Viewer m_viewer;
    igl::opengl::glfw::imgui::ImGuiPlugin m_plugin;
    igl::opengl::glfw::imgui::ImGuiMenu m_menu;

    Eigen::MatrixXd m_V, m_C;
    Eigen::MatrixXi m_F;
    Eigen::RowVector4f m_meshColor = Eigen::RowVector4f(0.8f, 0.8f, 0.8f, 1.0f);

    std::set<unsigned int> m_selectedIndexes;
    std::vector<std::set<int>> m_neighborsMap;
    bool m_nextIsSource = false;
    unsigned int m_source;
};