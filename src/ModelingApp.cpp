#include <ModelingApp.h>

#include <igl/unproject_onto_mesh.h>

ModelingApp::ModelingApp(Eigen::MatrixXd& V, Eigen::MatrixXi& F) : m_V(V), m_F(F)
{
    // Initialize white
    m_C = Eigen::MatrixXd::Constant(F.rows(),3,1);

    // Add callback for face selection
    addMouseCallback();

    // Set mesh data
    m_viewer.data().set_mesh(m_V, m_F);
    m_viewer.data().set_colors(m_C);
}

void ModelingApp::launch()
{
    this->viewer().launch();
}

igl::opengl::glfw::Viewer ModelingApp::viewer()
{
    return this->m_viewer;
}

void ModelingApp::addMouseCallback()
{
  m_viewer.callback_mouse_down =
        [this](igl::opengl::glfw::Viewer& viewer, int, int)->bool
    {
    int fid;
    Eigen::Vector3f bc;
    // Cast a ray in the view direction starting from the mouse position
    double x = viewer.current_mouse_x;
    double y = viewer.core().viewport(3) - viewer.current_mouse_y;
    if(igl::unproject_onto_mesh(Eigen::Vector2f(x,y),
      viewer.core().view,
      viewer.core().proj,
      viewer.core().viewport,
      m_V, m_F, fid, bc))
    {
      // Paint hit
      if(m_C(fid, 1) == 1) {
        m_C.row(fid) = Eigen::RowVector3d(1, 0, 0);
        m_selectedIndexes.insert(fid);
      } else {
        m_C.row(fid) = Eigen::RowVector3d(1, 1, 1);
        m_selectedIndexes.erase(fid);
      }
      viewer.data().set_colors(m_C);

      // Update selected indexes
      std::cout << "Currently selected : ";
      for(unsigned int id : m_selectedIndexes) {
        std::cout << id << " ";
      } std::cout << std::endl;

      return true;
    }
    return false;
  };
  std::cout<< R"(  [clic]  Pick face on shape)" << std::endl;
}
