#include "SimulationApp.hpp"
#include "CircleSphereNode.hpp"
#include "PendulumNode.hpp"
#include "ClothNodeSmooth.hpp"


#include "glm/gtx/string_cast.hpp"

#include "gloo/shaders/PhongShader.hpp"
#include "gloo/components/RenderingComponent.hpp"
#include "gloo/components/ShadingComponent.hpp"
#include "gloo/components/CameraComponent.hpp"
#include "gloo/components/LightComponent.hpp"
#include "gloo/components/MaterialComponent.hpp"
#include "gloo/MeshLoader.hpp"
#include "gloo/lights/PointLight.hpp"
#include "gloo/lights/AmbientLight.hpp"
#include "gloo/cameras/ArcBallCameraNode.hpp"
#include "gloo/debug/AxisNode.hpp"
#include "gloo/debug/PrimitiveFactory.hpp"



namespace GLOO {
SimulationApp::SimulationApp(const std::string& app_name,
                             glm::ivec2 window_size,
                             IntegratorType integrator_type,
                             float integration_step)
    : Application(app_name, window_size),
      integrator_type_(integrator_type),
      integration_step_(integration_step) {
  // TODO: remove the following two lines and use integrator type and step to
  // create integrators; the lines below exist only to suppress compiler
  // warnings.
}

void SimulationApp::SetupScene() {
  SceneNode& root = scene_->GetRootNode();

  auto camera_node = make_unique<ArcBallCameraNode>(45.f, 0.75f, 5.0f);
  scene_->ActivateCamera(camera_node->GetComponentPtr<CameraComponent>());
  root.AddChild(std::move(camera_node));

  root.AddChild(make_unique<AxisNode>('A'));

  auto ambient_light = std::make_shared<AmbientLight>();
  ambient_light->SetAmbientColor(glm::vec3(0.2f));
  root.CreateComponent<LightComponent>(ambient_light);

  auto point_light = std::make_shared<PointLight>();
  point_light->SetDiffuseColor(glm::vec3(0.8f, 0.8f, 0.8f));
  point_light->SetSpecularColor(glm::vec3(1.0f, 1.0f, 1.0f));
  point_light->SetAttenuation(glm::vec3(1.0f, 0.09f, 0.032f));
  auto point_light_node = make_unique<SceneNode>();
  point_light_node->CreateComponent<LightComponent>(point_light);
  point_light_node->GetTransform().SetPosition(glm::vec3(10.0f, 5.0f, 5.f));
  root.AddChild(std::move(point_light_node));


  auto color = glm::vec3(0.8f, 0.3f, 0.3f);
  // auto circle_sphere_node = make_unique<CircleSphereNode>(integrator_type_, color, integration_step_);
  // root.AddChild(std::move(circle_sphere_node));

  // auto pendulum_node = make_unique<PendulumNode>(integrator_type_, color, integration_step_);
  // root.AddChild(std::move(pendulum_node));

  auto cloth_node = make_unique<ClothNode>(integrator_type_, color, integration_step_);
  root.AddChild(std::move(cloth_node));

  auto color2 = glm::vec3(0.3f, 0.8f, 0.3f);

  auto cloth_node2 = make_unique<ClothNodeSmooth>(integrator_type_, color,glm::vec3(1.0,1.0,0.0), integration_step_, 8);
  root.AddChild(std::move(cloth_node2));


}
}  // namespace GLOO
