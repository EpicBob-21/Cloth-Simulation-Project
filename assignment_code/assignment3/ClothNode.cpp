#include "ClothNode.hpp"
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

    ClothNode::ClothNode(IntegratorType integrator_type, glm::vec3 color, float h) :
        SceneNode(),
        integrator_(IntegratorFactory::CreateIntegrator<PendulumSystem, ParticleState>(integrator_type)),
        state_(make_unique<ParticleState>()),
        system_(make_unique<PendulumSystem>()),
        h_(h){

        shader_ = std::make_shared<PhongShader>();
        point_mesh_ = PrimitiveFactory::CreateSphere(0.06f, 25, 25);
        material_ = std::make_shared<Material>(color, color, color, 0);
        line_material_ = std::make_shared<Material>(glm::vec3(1.0f, 1.0f, 1.0f), glm::vec3(1.0f, 1.0f, 1.0f), glm::vec3(1.0f, 1.0f, 1.0f), 0);

        system_->FixMass(0);
        system_->FixMass(7);
        state_->positions.resize(64);
        state_->velocities.resize(64);

        auto indices = make_unique<IndexArray>();

        for (float i = 0; i < 8; i++) {
            for (float j = 0; j < 8; j++) {
                float r = 1.0f;

                system_->AddMass(i*8 + j, r, 0.088f);

                if (i > 0) {
                    // strucutral
                    system_->AddSpring((i - 1)*8 + j, i*8 + j, 1000.0f, r);
                    //vertical line
                    indices->push_back((i - 1)*8 + j);
                    indices->push_back(i*8 + j);
                    if (i > 1) {
                        // flexion
                        system_->AddSpring((i - 2)*8 + j, i*8 + j, 1000.0f, 2*r);
                    }
                }

                if (j > 0) {
                    // structural
                    system_->AddSpring(i*8 + (j - 1), i*8 + j, 1000.0f, r);

                    // horizontal line
                    indices->push_back(i*8 + (j - 1));
                    indices->push_back(i*8 + j);
                    if (j > 1) {
                        // flexion
                        system_->AddSpring(i*8 + (j - 2), i*8 + j, 1000.0f, 2*r);
                    }
                }

                if (i > 0 && j > 0) {
                    //shear
                    system_->AddSpring((i - 1)*8 + (j - 1), i*8 + j, 10000.0f, glm::sqrt(2*r));
                }

                auto point_node_ptr = make_unique<SceneNode>();
                point_node_ptr->CreateComponent<MaterialComponent>(material_);
                point_node_ptr->CreateComponent<ShadingComponent>(shader_);
                auto& rc = point_node_ptr->CreateComponent<RenderingComponent>(point_mesh_);
                rc.SetDrawMode(DrawMode::Triangles);

                points_.push_back(point_node_ptr.get());
                AddChild(std::move(point_node_ptr));
            }
        }
        all_line_ = std::make_shared<VertexObject>();
        all_line_->UpdateIndices(std::move(indices));

        Restart();

        auto line_node = make_unique<SceneNode>();
        line_node->CreateComponent<ShadingComponent>(shader_);
        line_node->CreateComponent<MaterialComponent>(line_material_);

        auto& rc = line_node->CreateComponent<RenderingComponent>(all_line_);
        rc.SetDrawMode(DrawMode::Lines);
        AddChild(std::move(line_node));

    }

    void ClothNode::Update(double dt) {
        *state_ = integrator_->Integrate(*system_, *state_, 0.0f, h_);
        static bool restart_prev_released = true;
        if (InputManager::GetInstance().IsKeyPressed('R')) {
            restart_prev_released = false;
        } else if (InputManager::GetInstance().IsKeyReleased('R')) {
            if (!restart_prev_released) {
                Restart();
            }
            restart_prev_released = true;
        }

        static bool wind_prev_released = true;
        if (InputManager::GetInstance().IsKeyPressed('W')) {
            wind_prev_released = false;
        } else if (InputManager::GetInstance().IsKeyReleased('W')) {
            if (!wind_prev_released) {
                system_->Blow();
            }
            wind_prev_released = true;
        }

        auto positions = make_unique<PositionArray>();
        auto normals = make_unique<NormalArray>();
        for (int i = 0; i < points_.size(); i++) {
            points_[i]->GetTransform().SetPosition(state_->positions[i]);
            positions->push_back(state_->positions[i]);

            if (i == 0) {
                normals->push_back(glm::normalize(glm::cross(
                    state_->positions[i + 8] - state_->positions[i],
                    state_->positions[i + 1] - state_->positions[i]
                )));
            } else if (i < 7) {
                normals->push_back(glm::normalize(glm::cross(
                    state_->positions[i + 8] - state_->positions[i],
                    state_->positions[i - 1] - state_->positions[i]
                )));
            } else if (i % 8 == 0) {
                normals->push_back(glm::normalize(glm::cross(
                    state_->positions[i - 8] - state_->positions[i],
                    state_->positions[i + 1] - state_->positions[i]
                )));
            } else {
                normals->push_back(glm::normalize(glm::cross(
                    state_->positions[i - 8] - state_->positions[i],
                    state_->positions[i - 1] - state_->positions[i]
                )));
            }
        }
        all_line_->UpdatePositions(std::move(positions));
        all_line_->UpdateNormals(std::move(normals));
    }

    void ClothNode::Restart() {
        auto positions = make_unique<PositionArray>();
        auto normals = make_unique<NormalArray>();

        for (float i = 0; i < 8; i++) {
            for (float j = 0; j < 8; j++) {
                // updating state's x and v
                state_->positions[i*8+j] = (glm::vec3(j + 2.4f, 0.0f, i));
                state_->velocities[i*8+j] = (glm::vec3(0.0f, 0.0f, 0.0f));

                // for lines
                positions->push_back(glm::vec3(j + 2.4f, 0.0f, i));
                normals->push_back(glm::vec3(0.0f, 1.0f, 0.0f));

                float r = 1.0f;

                points_[i*8+j]->GetTransform().SetPosition(state_->positions[i]);
            }
        }

        all_line_->UpdatePositions(std::move(positions));
        all_line_->UpdateNormals(std::move(normals));

    }
}
