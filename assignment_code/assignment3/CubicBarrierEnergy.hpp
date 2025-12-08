// #ifndef CUBIC_BARRIER_ENERGY_H_
// #define CUBIC_BARRIER_ENERGY_H_

// #include <vector>
// #include <stdexcept>
// #include "ParticleState.hpp"
// #include "SparseMatrix.hpp"
// #include "EnergyTerm.hpp"

// #include <glm/glm.hpp>

// namespace GLOO {
// class CubicBarrierEnergy : public EnergyTerm {
// protected:
//     float g_hat_;  // Maximum gap distance
    
//     // Equation (1): weak cubic barrier
//     float PsiWeak(float g, float kappa_bar) const {
//         if (g <= g_hat_) {
//             float diff = g - g_hat_;
//             return -(2.0f * kappa_bar / (3.0f * g_hat_)) * diff * diff * diff;
//         }
//         return 0.0f;
//     }
    
//     // First derivative
//     float PsiWeakGradient(float g, float kappa_bar) const {
//         if (g <= g_hat_) {
//             float diff = g - g_hat_;
//             return -(2.0f * kappa_bar / g_hat_) * diff * diff;
//         }
//         return 0.0f;
//     }
    
//     // Second derivative (curvature)
//     float PsiWeakHessian(float g, float kappa_bar) const {
//         if (g <= g_hat_) {
//             float diff = g - g_hat_;
//             return -(4.0f * kappa_bar / g_hat_) * diff;
//         }
//         return 0.0f;
//     }
// };

// }  // namespace GLOO

// #endif


