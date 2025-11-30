#ifndef BALL_NODE_H_
#define BALL_NODE_H_

#include "gloo/SceneNode.hpp"
#include "IntegratorType.hpp"
#include "IntegratorBase.hpp"
#include "gloo/utils.hpp"
#include "gloo/VertexObject.hpp"
#include "gloo/debug/PrimitiveFactory.hpp"
#include "gloo/components/RenderingComponent.hpp"
#include "gloo/components/ShadingComponent.hpp"
#include "gloo/components/MaterialComponent.hpp"
#include "gloo/shaders/PhongShader.hpp"
#include "gloo/shaders/SimpleShader.hpp"
#include "gloo/utils.hpp"
#include "gloo/InputManager.hpp"
#include "gloo/MeshLoader.hpp"

#include "ParticleState.hpp"
#include "BallSystem.hpp"
#include "ForwardEulerIntegrator.hpp"
#include "IntegratorFactory.hpp"

namespace GLOO {
    class BallNode : public SceneNode {
        public:
            SceneNode* sphere_node_ptr;
            float step_size;
            float time;
            ParticleState sphere_state;
            BallSystem this_system;
            using Integrator = IntegratorBase<BallSystem, ParticleState>;
            std::unique_ptr<Integrator> this_integrator;

            BallNode(IntegratorType integrator_type,
                             float integration_step){

                step_size = integration_step;
                time = 0.f;

                //make ball
                std::shared_ptr<PhongShader> shader = std::make_shared<PhongShader>();
                std::shared_ptr<VertexObject> sphere_mesh_ = PrimitiveFactory::CreateSphere(0.08f, 24, 24);

                auto sphere_node = make_unique<SceneNode>();
                sphere_node->CreateComponent<ShadingComponent>(shader);
                auto& rc = sphere_node->CreateComponent<RenderingComponent>(sphere_mesh_);
                sphere_node_ptr = sphere_node.get();
                sphere_node_ptr->GetTransform().SetPosition(glm::vec3(1.f, 0.f, 3.f));
                sphere_node->CreateComponent<MaterialComponent>(std::make_shared<Material>(glm::vec3(1.0f, 0.4f, 0.7f), glm::vec3(1.0f, 0.4f, 0.7f), glm::vec3(1.0f, 0.4f, 0.7f),32.0f));
                AddChild(std::move(sphere_node));

                //set initial state
                sphere_state.positions.push_back(glm::vec3(1.f, 0.f, 3.f));
                sphere_state.velocities.push_back(glm::vec3(0.0f, 0.f, 0.f));

                //set up integrator
                this_integrator = IntegratorFactory::CreateIntegrator<BallSystem, ParticleState>(integrator_type);

            }
            void Update(double delta_time) override{
                float remaining_time = static_cast<float>(delta_time);
                //change state

                while(remaining_time>0){
                    sphere_state = this_integrator -> Integrate(this_system, sphere_state, time, step_size);
                    sphere_node_ptr -> GetTransform().SetPosition(sphere_state.positions[0]);
                    time += step_size;
                    remaining_time -= step_size;
                }

            }
    };
}
#endif
