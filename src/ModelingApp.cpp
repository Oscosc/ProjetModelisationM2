#include <ModelingApp.h>

#include <igl/unproject_onto_mesh.h>

ModelingApp::ModelingApp(Eigen::MatrixXd& V, Eigen::MatrixXi& F, const bool withMenu) : m_object(MeshObject(V, F))
{
    // Add callback for face selection
    addMouseCallback();

    // Add menu if wanted
    if (withMenu) addMenu();

    // Set mesh data in the viewer
    m_viewer.data().set_mesh(m_object.V(), m_object.F());
    m_viewer.data().set_colors(m_object.C());
}

void ModelingApp::launch()
{
    this->viewer().launch();
}

igl::opengl::glfw::Viewer& ModelingApp::viewer()
{
    return this->m_viewer;
}

void ModelingApp::addMouseCallback()
{
  /**
   * Callback for selecting a vertice on the mesh using mouse click
   * 
   */
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
      m_object.V(), m_object.F(), fid, bc))
    {
      // Get closest vertex ID
      int maxc;
      bc.maxCoeff(&maxc);
      int vid = m_object.F()(fid, maxc);

      // Paint hit
      updateVertexColor(vid);
      m_viewer.data().set_colors(m_object.C());

      // Update selected indexes
      std::cout << "Selection contain " << m_object.Laplacian().selected.size() << " vertices" << std::endl;
      return true;
    }
    return false;
  };
  std::cout<< R"(  [clic]  Pick vertex on shape)" << std::endl;
}

void ModelingApp::addMenu()
{
  // Ajout des plugins au viewer
  m_viewer.plugins.push_back(&m_plugin);
  m_plugin.widgets.push_back(&m_menu);
  
  // Add content to the default menu window
  m_menu.callback_draw_viewer_menu = [&]()
  {

    // Mesh
    if (ImGui::CollapsingHeader("Mesh", ImGuiTreeNodeFlags_DefaultOpen))
    {
      float w = ImGui::GetContentRegionAvail().x;
      float p = ImGui::GetStyle().FramePadding.x;
      if (ImGui::Button("Load##Mesh", ImVec2((w-p)/2.f, 0)))
      {
        m_viewer.open_dialog_load_mesh();
      }
      ImGui::SameLine(0, p);
      if (ImGui::Button("Save##Mesh", ImVec2((w-p)/2.f, 0)))
      {
        m_viewer.open_dialog_save_mesh();
      }
    }

    // Viewing options
    if (ImGui::CollapsingHeader("Viewing Options", ImGuiTreeNodeFlags_DefaultOpen))
    {
      if (ImGui::Button("Center object", ImVec2(-1, 0)))
      {
        m_viewer.core().align_camera_center(m_viewer.data().V, m_viewer.data().F);
      }
      if (ImGui::Button("Snap canonical view", ImVec2(-1, 0)))
      {
        m_viewer.snap_to_canonical_quaternion();
      }

      // Zoom
      ImGui::PushItemWidth(80 * m_menu.menu_scaling());
      ImGui::DragFloat("Zoom", &(m_viewer.core().camera_zoom), 0.05f, 0.1f, 20.0f);

      // Select rotation type
      int rotation_type = static_cast<int>(m_viewer.core().rotation_type);
      static Eigen::Quaternionf trackball_angle = Eigen::Quaternionf::Identity();
      static bool orthographic = true;
      if (ImGui::Combo("Camera Type", &rotation_type, "Trackball\0Two Axes\0002D Mode\0\0"))
      {
        using RT = igl::opengl::ViewerCore::RotationType;
        auto new_type = static_cast<RT>(rotation_type);
        if (new_type != m_viewer.core().rotation_type)
        {
          if (new_type == RT::ROTATION_TYPE_NO_ROTATION)
          {
            trackball_angle = m_viewer.core().trackball_angle;
            orthographic = m_viewer.core().orthographic;
            m_viewer.core().trackball_angle = Eigen::Quaternionf::Identity();
            m_viewer.core().orthographic = true;
          }
          else if (m_viewer.core().rotation_type == RT::ROTATION_TYPE_NO_ROTATION)
          {
            m_viewer.core().trackball_angle = trackball_angle;
            m_viewer.core().orthographic = orthographic;
          }
          m_viewer.core().set_rotation_type(new_type);
        }
      }

      // Orthographic view
      ImGui::Checkbox("Orthographic view", &(m_viewer.core().orthographic));
      ImGui::PopItemWidth();
    }

    // Helper for setting viewport specific mesh options
    auto make_checkbox = [&](const char *label, unsigned int &option)
    {
      return ImGui::Checkbox(label,
        [&]() { return m_viewer.core().is_set(option); },
        [&](bool value) { return m_viewer.core().set(option, value); }
      );
    };

    // Draw options
    if (ImGui::CollapsingHeader("Draw Options", ImGuiTreeNodeFlags_DefaultOpen))
    {
      if (ImGui::Checkbox("Face-based", &(m_viewer.data().face_based)))
      {
        m_viewer.data().dirty = igl::opengl::MeshGL::DIRTY_ALL;
      }
      make_checkbox("Show texture", m_viewer.data().show_texture);
      if (ImGui::Checkbox("Invert normals", &(m_viewer.data().invert_normals)))
      {
        m_viewer.data().dirty |= igl::opengl::MeshGL::DIRTY_NORMAL;
      }
      make_checkbox("Show overlay", m_viewer.data().show_overlay);
      make_checkbox("Show overlay depth", m_viewer.data().show_overlay_depth);
      ImGui::ColorEdit4("Background", m_viewer.core().background_color.data(),
          ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_PickerHueWheel);
      ImGui::ColorEdit4("Line color", m_viewer.data().line_color.data(),
          ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_PickerHueWheel);
      ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.3f);
      ImGui::DragFloat("Shininess", &(m_viewer.data().shininess), 0.05f, 0.0f, 100.0f);
      ImGui::PopItemWidth();
    }

    // Overlays
    if (ImGui::CollapsingHeader("Overlays", ImGuiTreeNodeFlags_DefaultOpen))
    {
      make_checkbox("Wireframe", m_viewer.data().show_lines);
      make_checkbox("Fill", m_viewer.data().show_faces);
      make_checkbox("Show vertex labels", m_viewer.data().show_vertex_labels);
      make_checkbox("Show faces labels", m_viewer.data().show_face_labels);
      make_checkbox("Show extra labels", m_viewer.data().show_custom_labels);
    }

    // Modeling
    if (ImGui::CollapsingHeader("Modeling Options", ImGuiTreeNodeFlags_DefaultOpen))
    {
      if (ImGui::Button("Add topological ring", ImVec2(-1, 0)))
      {
        if (!m_object.Laplacian().selected.empty()) {
          std::set<unsigned int> oldIndexes = m_object.Laplacian().selected;

          for(unsigned int index : oldIndexes) {
            const std::vector<int> nghbs = MeshOperators::getNeighbors(m_object.L(), index);
            for (int n : nghbs) {
              updateVertexColor(n, false);
            }
          }
        }
        m_viewer.data().set_colors(m_object.C());
        std::cout << "Selection contain " << m_object.Laplacian().selected.size() << " vertices" << std::endl;
      }

      if (ImGui::Button("Select source point", ImVec2(-1, 0)))
      {
        m_nextIsSource = true;
        std::cout << "Next point selected will be the new source !" << std::endl;
      }

      if(ImGui::Button("Reset selected vertices", ImVec2(-1, 0)))
      {
        m_object.resetLaplacianArea();
        m_viewer.data().set_colors(m_object.C());
        std::cout << "Selected vertices are now empty" << std::endl;
      }

      if(ImGui::Button("Init Laplacian", ImVec2(-1, 0)))
      {
        m_object.initLaplacian();
        setColorBasedOnLaplacian();
      }

      if(ImGui::Button("Step Laplacian", ImVec2(-1, 0)))
      {
        m_object.stepLaplacian();
        setColorBasedOnLaplacian();
      }

      if(ImGui::Button("Solve Laplacian", ImVec2(-1, 0)))
      {
        m_object.linearSolvingLaplacian();
        setColorBasedOnLaplacian();
      }

      if(ImGui::Button("+ Laplacian", ImVec2(-1, 0)))
      {
        m_object.deformLaplacian(0.01);
        m_viewer.data().set_vertices(m_object.V());
      }

      if(ImGui::Button("- Laplacian", ImVec2(-1, 0)))
      {
        m_object.deformLaplacian(-0.01);
        m_viewer.data().set_vertices(m_object.V());
      }
    }

  };
}

void ModelingApp::updateVertexColor(const unsigned int vid, const bool canDeactivate)
{
  bool isSelected = m_object.Laplacian().selected.count(vid);

  // Source point modification
  if(m_nextIsSource) {
    m_nextIsSource = false;
    m_object.changeLaplacianSource(vid);
    return;
  }

  // Other point modification
  if (!m_object.addLaplacian(vid) && canDeactivate) {
    m_object.removeLaplacian(vid);
  }
}

void ModelingApp::setColorBasedOnLaplacian() {
  Eigen::MatrixXd C(m_object.Laplacian().state.size(), 3);

  double minVal = m_object.Laplacian().state.minCoeff();
  double maxVal = m_object.Laplacian().state.maxCoeff();

  for (int i = 0; i < m_object.Laplacian().state.size(); ++i)
  {
      double t = (m_object.Laplacian().state[i] - minVal) / (maxVal - minVal + 1e-9); // normalisation
      // Bleu -> Rouge
      C(i, 0) = t;          // R
      C(i, 1) = 0.0;        // G
      C(i, 2) = 1.0 - t;    // B
  }

  m_viewer.data().set_colors(C);
}
