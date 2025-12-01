#include "PendulumNode.hpp"
#include "ParticleState.hpp"
#include "ForwardEulerIntegrator.hpp"
#include "ForwardTrapezoidIntegrator.hpp"
#include "IntegratorBase.hpp"
#include "IntegratorType.hpp"
#include "IntegratorFactory.hpp"
#include "PendulumSystem.hpp"


#include "gloo/shaders/PhongShader.hpp"
#include "gloo/components/RenderingComponent.hpp"
#include "gloo/components/ShadingComponent.hpp"
#include "gloo/components/MaterialComponent.hpp"
#include "gloo/InputManager.hpp"
#include "gloo/MeshLoader.hpp"
#include "gloo/debug/PrimitiveFactory.hpp"


// More include here
namespace GLOO {

    PendulumNode::PendulumNode(IntegratorType integrator_type, glm::vec3 color, float h) : 
        SceneNode(), 
        integrator_(IntegratorFactory::CreateIntegrator<PendulumSystem, ParticleState>(integrator_type)),
        state_(make_unique<ParticleState>()),
        system_(make_unique<PendulumSystem>()),
        h_(h) {

        shader_ = std::make_shared<PhongShader>();
        sphere_mesh_ = PrimitiveFactory::CreateSphere(0.1f, 25, 25);
        material_ = std::make_shared<Material>(color, color, color, 0);

        system_->FixMass(0);

        for (float i = 0; i < 4; i++) {
            // updating state's x and v
            state_->positions.push_back(glm::vec3(0.0f, -i, 0.0f));
            state_->velocities.push_back(glm::vec3(0.0f, 0.0f, 0.0f));

            // adding m, d, k, and r
            system_->AddMass(i, 1.0f, 0.088f);
            if (i > 0) {
                system_->AddSpring(i - 1, i, 25.0f, 1.0f);
            }

            auto sphere_node_ptr = make_unique<SceneNode>();
            sphere_node_ptr->CreateComponent<MaterialComponent>(material_);
            sphere_node_ptr->CreateComponent<ShadingComponent>(shader_);
            auto& rc = sphere_node_ptr->CreateComponent<RenderingComponent>(sphere_mesh_);
            rc.SetDrawMode(DrawMode::Triangles);

            spheres_.push_back(sphere_node_ptr.get());
            sphere_node_ptr->GetTransform().SetPosition(state_->positions[i]);
            AddChild(std::move(sphere_node_ptr));
        }
       
        
    }

    void PendulumNode::Update(double dt) {
        *state_ = integrator_->Integrate(*system_, *state_, 0.0f, h_);
        // std::cout << "Position: " << state_->positions[0].x << ", " << state_->positions[0].y << ", " << state_->positions[0].z << std::endl;
        for (int i = 0; i < spheres_.size(); i++) {
            spheres_[i]->GetTransform().SetPosition(state_->positions[i]);
        }
    }
}
