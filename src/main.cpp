#include <igl/readOBJ.h>
#include <igl/readOFF.h>

#include <ModelingApp.h>

int main(int argc, char *argv[])
{
  // Mesh with per-face color
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;

  // Load a mesh in OBJ format
  igl::readOFF("../resources/" + std::string(argv[1]), V, F);

  // Create App
  ModelingApp app(V, F);
  app.launch();
}
