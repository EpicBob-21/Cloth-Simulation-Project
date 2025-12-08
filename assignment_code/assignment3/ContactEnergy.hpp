// #ifndef CONTACT_ENERGY_H_
// #define CONTACT_ENERGY_H_

// #include <vector>
// #include <stdexcept>
// #include "ParticleState.hpp"
// #include "SparseMatrix.hpp"
// #include "CubicBarrierEnergy.hpp"
// #include "EnergyTerm.hpp"


// #include <glm/glm.hpp>

// namespace GLOO {

// enum class ContactType {
//     VERTEX_VERTEX,
//     VERTEX_EDGE,
//     VERTEX_TRIANGLE,
//     EDGE_EDGE
// };

// struct ContactPair {
//     int vertex_i;
//     int vertex_j;  // or triangle index for point-triangle
//     ContactType type;  // VERTEX_VERTEX, VERTEX_EDGE, VERTEX_TRIANGLE, EDGE_EDGE
//     glm::vec3 normal;
//     float distance;
//     float kappa_bar;  // Semi-implicit stiffness
// };

// class ContactEnergy : public CubicBarrierEnergy {
// private:
//     const MeshData& mesh_;
//     const SparseMatrix& elasticity_hessian_;
//     std::vector<ContactPair> contacts_;
    
// public:
//     ContactEnergy(const MeshData& mesh, 
//                  const SparseMatrix& H,
//                  float g_hat)
//         : mesh_(mesh), elasticity_hessian_(H) {
//         g_hat_ = g_hat;
//     }
    
//     void UpdateStiffness(const ParticleState& state) override {
//         // Equation (5): κ̄_contact = m/g² + w·(H·w)
//         for (auto& contact : contacts_) {
//             float g = contact.distance;
//             float m = mesh_.vertex_masses[contact.vertex_i];
            
//             // Extended contact direction w
//             glm::vec3 w = ComputeExtendedDirection(contact, state);
            
//             // Elasticity contribution: w·(H·w)
//             glm::vec3 Hw = elasticity_hessian_.Multiply(w);
//             float elasticity_stiffness = glm::dot(w, Hw);
            
//             // Combined stiffness
//             contact.kappa_bar = m / (g * g) + elasticity_stiffness;
//         }
//     }
    
//     float ComputeEnergy(const ParticleState& state) const override {
//         float energy = 0.0f;
//         for (const auto& contact : contacts_) {
//             float g = ComputeContactDistance(contact, state);
//             energy += PsiWeak(g, contact.kappa_bar);
//         }
//         return energy;
//     }
    
//     void AddGradient(const ParticleState& state,
//                     std::vector<glm::vec3>& gradient) const override {
//         for (const auto& contact : contacts_) {
//             float g = contact.distance;
//             float dPsi_dg = PsiWeakGradient(g, contact.kappa_bar);
            
//             // Chain rule: gradient w.r.t positions
//             AddContactGradientContribution(contact, dPsi_dg, gradient);
//         }
//     }
    
//     void AddHessian(const ParticleState& state,
//                    SparseMatrix& hessian) const override {
//         for (const auto& contact : contacts_) {
//             float g = contact.distance;
//             float d2Psi_dg2 = PsiWeakHessian(g, contact.kappa_bar);
            
//             // Chain rule: Hessian w.r.t positions
//             AddContactHessianContribution(contact, d2Psi_dg2, hessian);
//         }
//     }
    
// private:
//     glm::vec3 ComputeExtendedDirection(const ContactPair& contact,
//                                        const ParticleState& state) const {
//         // W_i^T * (p_i - q_i) from Section 3.4
//         // Implementation depends on contact type
//         return contact.normal; // Simplified
//     }
// };
// }  // namespace GLOO

// #endif
