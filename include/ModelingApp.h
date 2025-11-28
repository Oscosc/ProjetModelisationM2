#pragma once

#include <set>
#include <map>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <Eigen/Sparse>
#include <MeshOperators.h>
#include <MeshObject.h>

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

    void addPreDrawCallback();

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

    void setColorBasedOnLaplacian();

/***************************** ATTRIBUTES ****************************/
private:

    /** viewer object, used to display current mesh */
    igl::opengl::glfw::Viewer m_viewer;

    /** plugin object, used to link the menu to viewer */
    igl::opengl::glfw::imgui::ImGuiPlugin m_plugin;

    /** menu object, used to display various tools to perform functions on the mesh */
    igl::opengl::glfw::imgui::ImGuiMenu m_menu;

    /** INTERFACE : define if the next clicked point will set the source */
    bool m_nextIsSource = false;
    bool m_diffusing = false;

    /** Active object in the viewer */
    MeshObject m_object;
};