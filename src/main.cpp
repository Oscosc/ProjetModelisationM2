#include <igl/readOBJ.h>
#include <igl/readOFF.h>

#include <ModelingApp.h>

int main(int argc, char *argv[])
{
  // Mesh with per-face color
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;

  // Load a default mesh
  igl::readOFF("../resources/bunny.off", V, F);

  // Create App
  ModelingApp app(V, F);

  app.launch();
}
