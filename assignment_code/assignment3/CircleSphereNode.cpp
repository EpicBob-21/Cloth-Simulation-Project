#include "CircleSphereNode.hpp"
#include "ParticleState.hpp"
#include "ParticleSystemBase.hpp"
#include "ForwardEulerIntegrator.hpp"
#include "ForwardTrapezoidIntegrator.hpp"
#include "IntegratorBase.hpp"
#include "IntegratorType.hpp"
#include "IntegratorFactory.hpp"
#include "CircleSystemBase.hpp"


#include "gloo/shaders/PhongShader.hpp"
#include "gloo/components/RenderingComponent.hpp"
#include "gloo/components/ShadingComponent.hpp"
#include "gloo/components/MaterialComponent.hpp"
#include "gloo/InputManager.hpp"
#include "gloo/MeshLoader.hpp"
#include "gloo/debug/PrimitiveFactory.hpp"


// More include here
namespace GLOO {

    CircleSphereNode::CircleSphereNode(IntegratorType integrator_type, glm::vec3 color, float h) : 
        SceneNode(), 
        integrator_(IntegratorFactory::CreateIntegrator<CircleSystemBase, ParticleState>(integrator_type)),
        state_(make_unique<ParticleState>()),
        system_(make_unique<CircleSystemBase>()),
        h_(h) {

        shader_ = std::make_shared<PhongShader>();
        sphere_mesh_ = PrimitiveFactory::CreateSphere(0.1f, 25, 25);
        // auto color = glm::vec3(0.8f, 0.3f, 0.3f);
        material_ = std::make_shared<Material>(color, color, color, 0);

        auto& rc = CreateComponent<RenderingComponent>(sphere_mesh_);
        rc.SetDrawMode(DrawMode::Triangles);

        CreateComponent<ShadingComponent>(shader_);
        CreateComponent<MaterialComponent>(material_);

        state_->positions.push_back(glm::vec3(1.0f, 0.0f, 0.0f));
        state_->velocities.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
    }

    void CircleSphereNode::Update(double dt) {
        *state_ = integrator_->Integrate(*system_, *state_, 0.0f, h_);
        // std::cout << "Position: " << state_->positions[0].x << ", " << state_->positions[0].y << ", " << state_->positions[0].z << std::endl;
        GetTransform().SetPosition(state_->positions[0] - glm::vec3(2.4f, 0.0f, 0.0f));
    }
}
