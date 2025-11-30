#ifndef CLOTH_NODE_H_
#define CLOTH_NODE_H_

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
    class ClothNode : public SceneNode {
        public:
            std::vector<SceneNode*> sphere_node_ptrs;
            float step_size;
            float time;
            ParticleState sphere_state;
            PendulumSystem this_system;
            using Integrator = IntegratorBase<PendulumSystem, ParticleState>;
            std::unique_ptr<Integrator> this_integrator;
            int n;

            //cloth net
            std::shared_ptr<VertexObject> cloth_mesh_;
            SceneNode* cloth_node_ptr;

            ClothNode(IntegratorType integrator_type,
                             float integration_step){

                step_size = integration_step;
                time = 0.f;

                //make ball
                std::shared_ptr<PhongShader> shader = std::make_shared<PhongShader>();
                std::shared_ptr<VertexObject> sphere_mesh_ = PrimitiveFactory::CreateSphere(.08f, 24, 24);

                n = 8;
                float k = 1.f;

                for(int i=0;i<n;i++){
                    for(int j=0;j<n;j++){
                        auto sphere_node = make_unique<SceneNode>();
                        sphere_node->CreateComponent<ShadingComponent>(shader);
                        auto& rc = sphere_node->CreateComponent<RenderingComponent>(sphere_mesh_);
                        sphere_node_ptrs.push_back(sphere_node.get());
                        sphere_node_ptrs[IndexOf(i,j,n)]->GetTransform().SetPosition(glm::vec3(0.5f*i+3, -0.5f*j+2, 0.f));
                        sphere_node->CreateComponent<MaterialComponent>(std::make_shared<Material>(glm::vec3(0.0f, 0.6f, 1.0f), glm::vec3(0.0f, 0.8f, 1.0f), glm::vec3(0.0f, 0.8f, 1.0f), 32.0f));
                        AddChild(std::move(sphere_node));

                        //set initial state
                        sphere_state.positions.push_back(sphere_node_ptrs[IndexOf(i,j,n)] -> GetTransform().GetPosition());
                        sphere_state.velocities.push_back(glm::vec3(0.0f, 0.f, 0.f));
                    }
                }

                //connect adjacent spheres, structural springs
                for(int i=0;i<n-1;i++){
                    for(int j=0;j<n;j++){
                        sphere_state.springs.push_back(glm::vec4(IndexOf(i,j,n), IndexOf(i+1,j,n), k, 0.5f));
                    }
                }
                for(int i=0;i<n;i++){
                    for(int j=0;j<n-1;j++){
                        sphere_state.springs.push_back(glm::vec4(IndexOf(i,j,n), IndexOf(i,j+1,n), k, 0.5f));
                    }
                }

                //connect diagonals, shear springs

                for(int i=0;i<n-1;i++){
                    for(int j=0;j<n-1;j++){
                        sphere_state.springs.push_back(glm::vec4(IndexOf(i,j,n), IndexOf(i+1,j+1,n), k, 0.5f));
                    }
                }
                for(int i=1;i<n;i++){
                    for(int j=0;j<n-1;j++){
                        sphere_state.springs.push_back(glm::vec4(IndexOf(i,j,n), IndexOf(i-1,j+1,n), k, 0.5f));
                    }
                }

                //connect every second sphere, flex springs

                for(int i=0;i<n-2;i++){
                    for(int j=0;j<n;j++){
                        sphere_state.springs.push_back(glm::vec4(IndexOf(i,j,n), IndexOf(i+2,j,n), k, 1.f));
                    }
                }
                for(int i=1;i<n;i++){
                    for(int j=0;j<n-2;j++){
                        sphere_state.springs.push_back(glm::vec4(IndexOf(i,j,n), IndexOf(i,j+2,n), k, 1.f));
                    }
                }

                sphere_state.fixed_pts.push_back(0);
                sphere_state.fixed_pts.push_back(IndexOf(n-1,0,n));
                sphere_state.drag = 0.005f;


                //set up integrator
                this_integrator = IntegratorFactory::CreateIntegrator<PendulumSystem, ParticleState>(integrator_type);

                //set up cloth mesh
                printf("start mesh");
                cloth_mesh_ = std::make_shared<VertexObject>();

                auto line_positions = make_unique<PositionArray>();
                auto line_indices = make_unique<IndexArray>();

                for (int k=0;k<sphere_state.springs.size();k++) {
                    auto& s = sphere_state.springs[k];
                    int i = (int)s[0];
                    int j = (int)s[1];
                    line_positions->push_back(sphere_state.positions[i]);
                    line_positions->push_back(sphere_state.positions[j]);
                    line_indices->push_back(2*k);
                    line_indices->push_back(2*k+1);
                }

                printf("added coords");

                cloth_mesh_->UpdatePositions(std::move(line_positions));
                cloth_mesh_->UpdateIndices(std::move(line_indices));
                printf("updated coords");
                auto cloth_node = make_unique<SceneNode>();
                cloth_node_ptr = cloth_node.get();
                auto& cloth_rc = cloth_node->CreateComponent<RenderingComponent>(cloth_mesh_);
                cloth_rc.SetDrawMode(DrawMode::Lines);
                std::shared_ptr<SimpleShader> cloth_shader = std::make_shared<SimpleShader>();
                cloth_node->CreateComponent<ShadingComponent>(cloth_shader);
                AddChild(std::move(cloth_node));
                printf("finish cloth");

            }
            void Update(double delta_time) override{
                float remaining_time = static_cast<float>(delta_time);
                //change state

                while(remaining_time>0){
                    sphere_state = this_integrator -> Integrate(this_system, sphere_state, time, step_size);
                    for(int i=0;i<n;i++){
                        for(int j=0;j<n;j++){
                            sphere_node_ptrs[IndexOf(i,j,n)] -> GetTransform().SetPosition(sphere_state.positions[IndexOf(i,j,n)]);
                        }

                    }
                    auto line_positions = make_unique<PositionArray>();
                    auto line_indices = make_unique<IndexArray>();

                    for (int k=0;k<sphere_state.springs.size();k++) {
                        auto& s = sphere_state.springs[k];
                        int i = (int)s[0];
                        int j = (int)s[1];
                        line_positions->push_back(sphere_state.positions[i]);
                        line_positions->push_back(sphere_state.positions[j]);
                        line_indices->push_back(2*k);
                        line_indices->push_back(2*k+1);
                    }
                    cloth_mesh_->UpdatePositions(std::move(line_positions));
                    cloth_mesh_->UpdateIndices(std::move(line_indices));
                    time += step_size;
                    remaining_time -= step_size;
                }

                //reset cloth

                if (InputManager::GetInstance().IsKeyPressed('R')) {
                    sphere_state.positions.clear();
                    sphere_state.velocities.clear();
                    for(int i=0;i<n;i++){
                        for(int j=0;j<n;j++){
                            sphere_node_ptrs[IndexOf(i,j,n)]->GetTransform().SetPosition(glm::vec3(0.5f*i+3, -0.5f*j+2, 0.f));

                            //set initial state
                            sphere_state.positions.push_back(sphere_node_ptrs[IndexOf(i,j,n)] -> GetTransform().GetPosition());
                            sphere_state.velocities.push_back(glm::vec3(0.0f, 0.f, 0.f));
                        }
                    }
                }
            }

            int IndexOf(int i, int j, int n){
                return n*i+j;
            }
    };
}
#endif
