// #ifndef INERTIA_ENERGY_H_
// #define INERTIA_ENERGY_H_

// #include <vector>
// #include <stdexcept>
// #include "ParticleState.hpp"
// #include "SparseMatrix.hpp"
// #include "EnergyTerm.hpp"

// #include <glm/glm.hpp>

// namespace GLOO {
// class InertiaEnergy : public EnergyTerm {
// private:
//     const MeshData& mesh_;
//     float dt_;
    
// public:
//     InertiaEnergy(const MeshData& mesh, float dt) 
//         : mesh_(mesh), dt_(dt) {}
    
//     float ComputeEnergy(const ParticleState& state) const override {
//         // ψ_dyn = Σ (m_i / 2) * ||x_i - z_i||^2
//         float energy = 0.0f;
//         for (size_t i = 0; i < state.positions.size(); i++) {
//             glm::vec3 diff = state.positions[i] - state.inertia_targets[i];
//             energy += 0.5f * mesh_.vertex_masses[i] * glm::dot(diff, diff);
//         }
//         return energy;
//     }
    
//     void AddGradient(const ParticleState& state, 
//                     std::vector<glm::vec3>& gradient) const override {
//         // ∇ψ = m_i * (x_i - z_i)
//         for (size_t i = 0; i < state.positions.size(); i++) {
//             gradient[i] += mesh_.vertex_masses[i] * 
//                           (state.positions[i] - state.inertia_targets[i]);
//         }
//     }
    
//     void AddHessian(const ParticleState& state,
//                    SparseMatrix& hessian) const override {
//         // ∇²ψ = m_i * I (diagonal 3x3 blocks)
//         for (size_t i = 0; i < state.positions.size(); i++) {
//             float m = mesh_.vertex_masses[i];
//             hessian.AddDiagonalBlock(i, i, m * glm::mat3(1.0f));
//         }
//     }
    
//     void UpdateStiffness(const ParticleState& state) override {
//         // No stiffness update needed for inertia
//     }
// };

// }  // namespace GLOO

// #endif


