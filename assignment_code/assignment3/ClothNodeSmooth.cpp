#include "ClothNodeSmooth.hpp"
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

    ClothNodeSmooth::ClothNodeSmooth(IntegratorType integrator_type, glm::vec3 color, glm::vec3 top_left_pos, float h, int dim) :
        SceneNode(),
        integrator_(IntegratorFactory::CreateIntegrator<PendulumSystem, ParticleState>(integrator_type)),
        state_(make_unique<ParticleState>()),
        system_(make_unique<PendulumSystem>()),
        h_(h){

        shader_ = std::make_shared<PhongShader>();
        // point_mesh_ = PrimitiveFactory::CreateSphere(0.06f, 25, 25);
        material_ = std::make_shared<Material>(color, color, color, 0);
        // line_material_ = std::make_shared<Material>(glm::vec3(1.0f, 1.0f, 1.0f), glm::vec3(1.0f, 1.0f, 1.0f), glm::vec3(1.0f, 1.0f, 1.0f), 0);
        DIM = dim;
        top_left_corner_pos_ = top_left_pos;
        system_->FixMass(0);
        system_->FixMass(DIM-1);
        state_->positions.resize(DIM*DIM);
        state_->velocities.resize(DIM*DIM);

        auto indices = make_unique<IndexArray>();

        for (float i = 0; i < DIM; i++) {
            for (float j = 0; j < DIM; j++) {
                float r = 1.0f;

                system_->AddMass(i*DIM + j, r, 0.088f);

                if (i > 0) {
                    // strucutral
                    system_->AddSpring((i - 1)*DIM + j, i*DIM + j, 100.0f, r);
                    //vertical line
                    indices->push_back((i - 1)*DIM + j);
                    indices->push_back(i*DIM + j);
                    if (i > 1) {
                        // flexion
                        system_->AddSpring((i - 2)*DIM + j, i*DIM + j, 100.0f, 2*r);
                    }
                }

                if (j > 0) {
                    // structural
                    system_->AddSpring(i*DIM + (j - 1), i*DIM + j, 100.0f, r);

                    // horizontal line
                    // indices->push_back(i*8 + (j - 1));
                    // indices->push_back(i*8 + j);
                    if (j > 1) {
                        // flexion
                        system_->AddSpring(i*DIM + (j - 2), i*DIM + j, 100.0f, 2*r);
                    }
                }

                if (i > 0 && j > 0) {
                    //shear
                    system_->AddSpring((i - 1)*DIM + (j - 1), i*DIM + j, 1000.0f, glm::sqrt(2.0f*r));
                }

                // auto point_node_ptr = make_unique<SceneNode>();
                // point_node_ptr->CreateComponent<MaterialComponent>(material_);
                // point_node_ptr->CreateComponent<ShadingComponent>(shader_);
                // auto& rc = point_node_ptr->CreateComponent<RenderingComponent>(point_mesh_);
                // rc.SetDrawMode(DrawMode::Triangles);

                // points_.push_back(point_node_ptr.get());
                // AddChild(std::move(point_node_ptr));
            }
        }
        // all_line_ = std::make_shared<VertexObject>();
        // all_line_->UpdateIndices(std::move(indices));


        // auto line_node = make_unique<SceneNode>();
        // line_node->CreateComponent<ShadingComponent>(shader_);
        // line_node->CreateComponent<MaterialComponent>(line_material_);

        // auto& rc = line_node->CreateComponent<RenderingComponent>(all_line_);
        // rc.SetDrawMode(DrawMode::Lines);
        // AddChild(std::move(line_node));
        cloth_mesh_ = std::make_shared<VertexObject>();

        auto triangle_indices = make_unique<IndexArray>();

        // Create triangles for the cloth grid
        for (int i = 0; i < DIM - 1; i++) {
            for (int j = 0; j < DIM - 1; j++) {
                int v0 = i * DIM + j;       // Top-left
                int v1 = i * DIM + j + 1;   // Top-right
                int v2 = (i + 1) * DIM + j;   // Bottom-left
                int v3 = (i + 1) * DIM + j + 1; // Bottom-right

                // First triangle (Top-left, Top-right, Bottom-left)
                triangle_indices->push_back(v0);
                triangle_indices->push_back(v1);
                triangle_indices->push_back(v2);

                // Second triangle (Bottom-left, Top-right, Bottom-right)
                triangle_indices->push_back(v2);
                triangle_indices->push_back(v1);
                triangle_indices->push_back(v3);
            }
        }

        cloth_mesh_->UpdateIndices(std::move(triangle_indices));

        // Add the cloth mesh to the scene node
        auto cloth_node = make_unique<SceneNode>();
        cloth_node->CreateComponent<ShadingComponent>(shader_);
        cloth_node->CreateComponent<MaterialComponent>(material_);
        auto& rc = cloth_node->CreateComponent<RenderingComponent>(cloth_mesh_);
        rc.SetDrawMode(DrawMode::Triangles); // Render as a surface of triangles
        AddChild(std::move(cloth_node));
        Restart();

    }

    void ClothNodeSmooth::Update(double dt) {
        // 1. Physics Integration
        *state_ = integrator_->Integrate(*system_, *state_, 0.0f, h_);

        // 2. Input Handling
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

        // 3. Update Visuals (Smooth Cloth Mesh)

        // Create the arrays ONCE
        auto positions = make_unique<PositionArray>();
        auto normals = make_unique<NormalArray>();

        // Fill positions from particle state
        for (size_t i = 0; i < state_->positions.size(); i++) {
            positions->push_back(state_->positions[i]);
        }

        // Calculate Smooth Normals
        for (int i = 0; i < DIM; i++) {
            for (int j = 0; j < DIM; j++) {
                int current_idx = i * DIM + j;
                glm::vec3 normal = glm::vec3(0.0f);

                // Accumulate normals from adjacent triangles
                // Top-Left Quad Neighbor
                if (i < DIM - 1 && j < DIM - 1) {
                    glm::vec3 v0 = state_->positions[current_idx];
                    glm::vec3 v1 = state_->positions[current_idx + 1];
                    glm::vec3 v2 = state_->positions[current_idx + DIM];
                    normal += glm::normalize(glm::cross(v1 - v0, v2 - v0));
                }
                // Bottom-Left Quad Neighbor
                if (i > 0 && j < DIM - 1) {
                    glm::vec3 v0 = state_->positions[current_idx];
                    glm::vec3 v1 = state_->positions[current_idx + 1];
                    glm::vec3 v2 = state_->positions[current_idx - DIM];
                    normal += glm::normalize(glm::cross(v2 - v0, v1 - v0));
                }
                // Top-Right Quad Neighbor
                if (i < DIM - 1 && j > 0) {
                    glm::vec3 v0 = state_->positions[current_idx];
                    glm::vec3 v1 = state_->positions[current_idx - 1];
                    glm::vec3 v2 = state_->positions[current_idx + DIM];
                    normal += glm::normalize(glm::cross(v2 - v0, v1 - v0));
                }
                // Bottom-Right Quad Neighbor
                if (i > 0 && j > 0) {
                    glm::vec3 v0 = state_->positions[current_idx];
                    glm::vec3 v1 = state_->positions[current_idx - 1];
                    glm::vec3 v2 = state_->positions[current_idx - DIM];
                    normal += glm::normalize(glm::cross(v1 - v0, v2 - v0));
                }

                // Normalize the accumulated result
                if (glm::length(normal) > 0.0f) {
                    normals->push_back(glm::normalize(normal));
                } else {
                    normals->push_back(glm::vec3(0.0f, 1.0f, 0.0f));
                }
            }
        }

        // Push updates to the GPU
        cloth_mesh_->UpdatePositions(std::move(positions));
        cloth_mesh_->UpdateNormals(std::move(normals));
    }

    void ClothNodeSmooth::Restart() {
        auto positions = make_unique<PositionArray>();
        auto normals = make_unique<NormalArray>();

        for (int i = 0; i < DIM; i++) {
            for (int j = 0; j < DIM; j++) {
                int idx = i * DIM + j;

                // Reset State
                state_->positions[idx] = (glm::vec3(j + top_left_corner_pos_.x, top_left_corner_pos_.y, i+top_left_corner_pos_.z));
                state_->velocities[idx] = (glm::vec3(0.0f, 0.0f, 0.0f));

                // Reset Visuals for Cloth Mesh
                positions->push_back(state_->positions[idx]);
                normals->push_back(glm::vec3(0.0f, 1.0f, 0.0f)); // Initial flat up-normal
            }
        }

        cloth_mesh_->UpdatePositions(std::move(positions));
        cloth_mesh_->UpdateNormals(std::move(normals));
    }
}
