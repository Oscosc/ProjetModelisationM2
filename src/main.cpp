#include <igl/opengl/glfw/Viewer.h>
#include <igl/readOBJ.h>

#include <Utils.h>

int main(int argc, char *argv[])
{
  // Matrices
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;

  // Mesh loaded
  igl::readOBJ("../resources/bunny.obj", V, F);

  // Plot the mesh
  igl::opengl::glfw::Viewer viewer;
  viewer.data().set_mesh(V, F);
  viewer.data().set_face_based(true);

  printTest();

  // Lauching the viewer
  viewer.launch();
}
