// ClothSystem.hpp
#ifndef CLOTH_SYSTEM_V2_H_
#define CLOTH_SYSTEM_V2_H_

#include "ParticleSystemBase.hpp"
#include <Eigen/Sparse>
#include "ModifiedPCG.hpp"

namespace GLOO {

class ClothSystemV2 : public ParticleSystemBase {
private:
    struct Triangle {
        int v0, v1, v2;
        glm::vec2 uv0, uv1, uv2;  // Rest state UV coordinates
        float area;
    };
    
    struct Spring {
        int i, j;
        float rest_length;
        float stiffness;
    };
    
    std::vector<Triangle> triangles_;
    std::vector<Spring> structural_springs_;
    std::vector<Spring> shear_springs_;
    std::vector<Spring> bend_springs_;
    std::vector<float> masses_;
    std::vector<int> fixed_particles_;
    
    // Material properties
    float stretch_stiffness_ = 1000.0f;
    float shear_stiffness_ = 100.0f;
    float bend_stiffness_ = 10.0f;
    float damping_ = 0.1f;
    
public:
    // Keep for compatibility with Forward Euler
    ParticleState ComputeTimeDerivative(const ParticleState& state,
                                        float time) const override {
        // Simple forward Euler version
        ParticleState derivative;
        auto forces = ComputeForces(state);
        
        for (size_t i = 0; i < state.positions.size(); i++) {
            derivative.positions.push_back(state.velocities[i]);
            derivative.velocities.push_back(forces[i] / masses_[i]);
        }
        return derivative;
    }
    
    // NEW: Methods for Backward Euler
    std::vector<glm::vec3> ComputeForces(const ParticleState& state) const {
        std::vector<glm::vec3> forces(state.positions.size(), glm::vec3(0));
        
        // Gravity
        for (size_t i = 0; i < forces.size(); i++) {
            forces[i] += glm::vec3(0, -9.81f * masses_[i], 0);
        }
        
        // Stretch forces (Section 4.2)
        for (const auto& tri : triangles_) {
            AddStretchForce(tri, state, forces);
        }
        
        // Shear forces (Section 4.3)
        for (const auto& tri : triangles_) {
            AddShearForce(tri, state, forces);
        }
        
        // Bend forces (Section 4.3)
        for (size_t i = 0; i < bend_springs_.size(); i++) {
            AddBendForce(bend_springs_[i], state, forces);
        }
        
        // Damping (Section 4.5)
        AddDampingForces(state, forces);
        
        return forces;
    }
    
    // Compute ∂f/∂x (Section 4.1, Equation 8)
    Eigen::SparseMatrix<float> ComputeForceJacobian(
        const ParticleState& state) const {
        
        int n = state.positions.size();
        Eigen::SparseMatrix<float> K(3*n, 3*n);
        std::vector<Eigen::Triplet<float>> triplets;
        
        // For each energy term, compute K_ij = ∂f_i/∂x_j
        // Using Equation (8):
        // K_ij = -k(∂C/∂x_i * ∂C/∂x_j^T + ∂²C/∂x_i∂x_j * C(x))
        
        for (const auto& tri : triangles_) {
            AddStretchJacobian(tri, state, triplets);
            AddShearJacobian(tri, state, triplets);
        }
        
        for (const auto& spring : bend_springs_) {
            AddBendJacobian(spring, state, triplets);
        }
        
        K.setFromTriplets(triplets.begin(), triplets.end());
        return K;
    }
    
    // Compute ∂f/∂v (Section 4.5, Equation 11-12)
    Eigen::SparseMatrix<float> ComputeDampingJacobian(
        const ParticleState& state) const {
        
        int n = state.positions.size();
        Eigen::SparseMatrix<float> D(3*n, 3*n);
        std::vector<Eigen::Triplet<float>> triplets;
        
        // Damping jacobian from Equation (12)
        // (simplified version without the asymmetric term)
        
        for (const auto& tri : triangles_) {
            AddDampingJacobian(tri, state, triplets);
        }
        
        D.setFromTriplets(triplets.begin(), triplets.end());
        return D;
    }
    
    Eigen::SparseMatrix<float> GetMassMatrix() const {
        int n = masses_.size();
        Eigen::SparseMatrix<float> M(3*n, 3*n);
        std::vector<Eigen::Triplet<float>> triplets;
        
        for (int i = 0; i < n; i++) {
            for (int d = 0; d < 3; d++) {
                triplets.push_back(Eigen::Triplet<float>(
                    3*i + d, 3*i + d, masses_[i]));
            }
        }
        
        M.setFromTriplets(triplets.begin(), triplets.end());
        return M;
    }
    
    std::vector<Constraint> GetConstraints() const {
        std::vector<Constraint> constraints;
        for (int idx : fixed_particles_) {
            constraints.push_back({idx, 0}); // 0 DOF = fully constrained
        }
        return constraints;
    }
    
private:
    // Section 4.2: Stretch forces
    void AddStretchForce(const Triangle& tri,
                        const ParticleState& state,
                        std::vector<glm::vec3>& forces) const {
        // Compute w_u and w_v using Equation (9)
        glm::vec3 x1 = state.positions[tri.v1] - state.positions[tri.v0];
        glm::vec3 x2 = state.positions[tri.v2] - state.positions[tri.v0];
        
        // UV differences (constant, from rest state)
        glm::vec2 duv1 = tri.uv1 - tri.uv0;
        glm::vec2 duv2 = tri.uv2 - tri.uv0;
        
        // Compute (w_u, w_v) = (x1, x2) * [duv1, duv2]^-1
        float det = duv1.x * duv2.y - duv1.y * duv2.x;
        glm::mat2 uv_inv = glm::mat2(
            duv2.y, -duv1.y,
            -duv2.x, duv1.x
        ) / det;
        
        glm::vec3 w_u = x1 * uv_inv[0][0] + x2 * uv_inv[1][0];
        glm::vec3 w_v = x1 * uv_inv[0][1] + x2 * uv_inv[1][1];
        
        // Condition C(x) from Equation (10)
        glm::vec2 C(
            glm::length(w_u) - 1.0f,  // b_u = 1
            glm::length(w_v) - 1.0f   // b_v = 1
        );
        C *= tri.area;
        
        // Force from Equation (7): f_i = -k * ∂C/∂x_i * C(x)
        // Need to compute ∂C/∂x for each vertex
        // ... (detailed implementation)
    }
    
    void AddShearForce(const Triangle& tri,
                      const ParticleState& state,
                      std::vector<glm::vec3>& forces) const {
        // C(x) = a * w_u^T * w_v (Section 4.3)
        // ... implementation
    }
    
    void AddBendForce(const Spring& spring,
                     const ParticleState& state,
                     std::vector<glm::vec3>& forces) const {
        // Angle between adjacent triangles (Section 4.3)
        // ... implementation
    }
    
    void AddDampingForces(const ParticleState& state,
                         std::vector<glm::vec3>& forces) const {
        // Equation (11): d = -k_d * ∂C/∂x * Ċ(x)
        // ... implementation
    }
    
    void AddStretchJacobian(const Triangle& tri,
                           const ParticleState& state,
                           std::vector<Eigen::Triplet<float>>& triplets) const {
        // Equation (8) applied to stretch condition
        // ... implementation
    }
    
    void AddShearJacobian(const Triangle& tri,
                         const ParticleState& state,
                         std::vector<Eigen::Triplet<float>>& triplets) const {
        // Equation (8) applied to shear condition
        // ... implementation
    }
    
    void AddBendJacobian(const Spring& spring,
                        const ParticleState& state,
                        std::vector<Eigen::Triplet<float>>& triplets) const {
        // Equation (8) applied to bend condition
        // ... implementation
    }
    
    void AddDampingJacobian(const Triangle& tri,
                           const ParticleState& state,
                           std::vector<Eigen::Triplet<float>>& triplets) const {
        // Equation (12) (simplified) applied to damping
        // ... implementation
    }
};

}  // namespace GLOO

#endif