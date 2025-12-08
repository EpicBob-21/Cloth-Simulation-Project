// // NewtonSolver.hpp
// #ifndef NEWTON_SOLVER_H_
// #define NEWTON_SOLVER_H_

// #include "ParticleState.hpp"
// #include "ClothSystem.hpp"
// #include "EnergyTerm.hpp"
// #include <vector>
// #include <memory>

// namespace GLOO {

// class NewtonSolver {
// private:
//     float beta_max_;
//     int max_newton_steps_;
    
//     // Energy terms
//     std::unique_ptr<InertiaEnergy> inertia_energy_;
//     std::unique_ptr<ElasticityEnergy> elasticity_energy_;
//     std::unique_ptr<ContactEnergy> contact_energy_;
//     std::unique_ptr<StrainLimitingEnergy> strain_limiting_energy_;
    
// public:
//     NewtonSolver() : beta_max_(0.25f), max_newton_steps_(10) {}
    
//     ParticleState AdvanceStep(const ClothSystem& system,
//                              const ParticleState& state,
//                              float dt) const {
//         ParticleState new_state = state;
        
//         // Update inertia targets: z = x + dt*v
//         new_state.inertia_targets.resize(state.positions.size());
//         for (size_t i = 0; i < state.positions.size(); i++) {
//             new_state.inertia_targets[i] = state.positions[i] + 
//                                            dt * state.velocities[i];
//         }
        
//         // Newton iteration (Algorithm 1)
//         float beta = 0.0f;
//         int iter = 0;
        
//         while (beta < beta_max_ && iter < max_newton_steps_) {
//             float alpha = InnerStep(system, new_state, dt);
//             beta = beta + (1.0f - beta) * alpha;
//             iter++;
//         }
        
//         // Error reduction pass
//         InnerStep(system, new_state, beta * dt);
        
//         // Update velocities from position change
//         for (size_t i = 0; i < new_state.positions.size(); i++) {
//             new_state.velocities[i] = 
//                 (new_state.positions[i] - state.positions[i]) / dt;
//         }
        
//         new_state.prev_positions = state.positions;
        
//         return new_state;
//     }
    
// private:
//     float InnerStep(const ClothSystem& system,
//                    ParticleState& state,
//                    float dt) const {
//         // 1. Update semi-implicit stiffnesses
//         UpdateStiffnesses(system, state);
        
//         // 2. Assemble system: H*d = -gradient
//         SparseMatrix H = AssembleHessian(system, state, dt);
//         std::vector<glm::vec3> gradient = AssembleGradient(system, state, dt);
        
//         // 3. Solve for search direction
//         std::vector<glm::vec3> d = SolvePCG(H, gradient);
        
//         // 4. Extend search direction by 25%
//         for (auto& di : d) {
//             di *= 1.25f;
//         }
        
//         // 5. Line search for safe step size
//         float alpha = LineSearch(system, state, d);
        
//         // 6. Update positions
//         for (size_t i = 0; i < state.positions.size(); i++) {
//             state.positions[i] += alpha * d[i];
//         }
        
//         return alpha;
//     }
    
//     void UpdateStiffnesses(const ClothSystem& system,
//                           const ParticleState& state) const {
//         // Update κ̄ for all energy terms
//         // This is where Equations (5), (6), (7), (9) are computed
//         // Details in next section...
//     }
    
//     SparseMatrix AssembleHessian(const ClothSystem& system,
//                                 const ParticleState& state,
//                                 float dt) const {
//         // Assemble global Hessian matrix
//         // Details in next section...
//     }
    
//     std::vector<glm::vec3> AssembleGradient(const ClothSystem& system,
//                                            const ParticleState& state,
//                                            float dt) const {
//         // Assemble global gradient vector
//         // Details in next section...
//     }
    
//     std::vector<glm::vec3> SolvePCG(const SparseMatrix& H,
//                                    const std::vector<glm::vec3>& b) const {
//         // Preconditioned Conjugate Gradient solver
//         // Details in next section...
//     }
    
//     float LineSearch(const ClothSystem& system,
//                     const ParticleState& state,
//                     const std::vector<glm::vec3>& direction) const {
//         // Line search with collision detection
//         // Start with alpha = 1.0, reduce if collisions detected
//         float alpha = 1.0f;
//         // ... collision checking ...
//         return alpha;
//     }
// };

// }  // namespace GLOO

// #endif