#pragma once

#include <igl/opengl/glfw/Viewer.h>
#include <set>

class ModelingApp : public igl::opengl::glfw::Viewer
{
/****************************** METHODS ******************************/
public:
    ModelingApp(Eigen::MatrixXd& V, Eigen::MatrixXi& F);
    void launch();

    igl::opengl::glfw::Viewer viewer();

private:
    void addMouseCallback();

/***************************** ATTRIBUTES ****************************/
private:
    igl::opengl::glfw::Viewer m_viewer;

    Eigen::MatrixXd m_V, m_C;
    Eigen::MatrixXi m_F;

    std::set<unsigned int> m_selectedIndexes;
};