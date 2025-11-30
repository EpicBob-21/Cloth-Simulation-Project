#ifndef PENDULUM_NODE_H_
#define PENDULUM_NODE_H_

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
#include "PendulumSystem.hpp"
#include "ForwardEulerIntegrator.hpp"
#include "IntegratorFactory.hpp"

namespace GLOO {
    class PendulumNode : public SceneNode {
        public:
            std::vector<SceneNode*> sphere_node_ptrs;
            float step_size;
            float time;
            ParticleState sphere_state;
            PendulumSystem this_system;
            using Integrator = IntegratorBase<PendulumSystem, ParticleState>;
            std::unique_ptr<Integrator> this_integrator;

            PendulumNode(IntegratorType integrator_type,
                             float integration_step){

                step_size = integration_step;
                time = 0.f;

                //make ball
                std::shared_ptr<PhongShader> shader = std::make_shared<PhongShader>();
                std::shared_ptr<VertexObject> sphere_mesh_ = PrimitiveFactory::CreateSphere(0.2f, 24, 24);

                for(int i=0;i<4;i++){
                    auto sphere_node = make_unique<SceneNode>();
                    sphere_node->CreateComponent<ShadingComponent>(shader);
                    auto& rc = sphere_node->CreateComponent<RenderingComponent>(sphere_mesh_);
                    sphere_node_ptrs.push_back(sphere_node.get());
                    sphere_node_ptrs[i]->GetTransform().SetPosition(glm::vec3(2.f*i, -.03f*i+2, 0.f));
                    sphere_node->CreateComponent<MaterialComponent>(std::make_shared<Material>(Material::GetDefault()));
                    AddChild(std::move(sphere_node));

                    //set initial state
                    sphere_state.positions.push_back(glm::vec3(2.f*i, -.03f*i+2, 0.f));
                    sphere_state.velocities.push_back(glm::vec3(0.0f, 0.f, 0.f));
                    if(i>0){
                        sphere_state.springs.push_back(glm::vec4((float)i, (float)(i - 1),0.05f, 0.5f));
                    }
                }

                sphere_state.fixed_pts.push_back(0);
                sphere_state.drag = 0.0005f;


                //set up integrator
                this_integrator = IntegratorFactory::CreateIntegrator<PendulumSystem, ParticleState>(integrator_type);

                //think about adding more springs than just the ones connecting consecutive balls?

            }
            void Update(double delta_time) override{
                float remaining_time = static_cast<float>(delta_time);
                //change state

                while(remaining_time>0){
                    sphere_state = this_integrator -> Integrate(this_system, sphere_state, time, step_size);
                    for(int i=0;i<4;i++){
                        sphere_node_ptrs[i] -> GetTransform().SetPosition(sphere_state.positions[i]);
                    }
                    time += step_size;
                    remaining_time -= step_size;
                }
            }
    };
}
#endif
