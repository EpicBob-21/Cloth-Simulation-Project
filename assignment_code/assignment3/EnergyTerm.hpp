// #ifndef ENERGY_TERM_H_
// #define ENERGY_TERM_H_

// #include <vector>
// #include <stdexcept>
// #include "ParticleState.hpp"
// #include "SparseMatrix.hpp"

// #include <glm/glm.hpp>

// namespace GLOO {
// class EnergyTerm {
// public:
//     virtual ~EnergyTerm() = default;
    
//     // Compute energy value
//     virtual float ComputeEnergy(const ParticleState& state) const = 0;
    
//     // Compute gradient (force): -∇ψ
//     virtual void AddGradient(const ParticleState& state, 
//                             std::vector<glm::vec3>& gradient) const = 0;
    
//     // Compute Hessian and add to sparse matrix
//     virtual void AddHessian(const ParticleState& state,
//                            SparseMatrix& hessian) const = 0;
    
//     // Update semi-implicit stiffness (κ_bar)
//     virtual void UpdateStiffness(const ParticleState& state) = 0;
// }; 

// }  // namespace GLOO

// #endif


