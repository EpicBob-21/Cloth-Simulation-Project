// ClothSystem.hpp
#ifndef CLOTH_SYSTEM_V2_H_
#define CLOTH_SYSTEM_V2_H_

#include "ParticleSystemBase.hpp"
#include <Eigen/Sparse>
#include "ModifiedPCG.hpp"
#include <iostream>

namespace GLOO {

class ClothSystemV2 : public ParticleSystemBase {
private:
    struct Triangle {
        int v0, v1, v2;
        glm::vec2 uv0, uv1, uv2;  // Rest state UV coordinates
        float area;
        glm::mat2 uv_inv;
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
    bool wind_ = false;
    
    // Material properties
    float stretch_stiffness_ = 1000.0f;
    float shear_stiffness_ = 100.0f;
    // float bend_stiffness_ = 10.0f;
    float damping_stiffness_ = 0.1f;
    // float damping_ = 0.1f;

    
public:
    std::vector<glm::vec3> ComputeDeltaVelocity(
        const ParticleState& state,
        float dt, 
        const ModifiedPCG& pcg_solver
    ) const {
            // Evaluate at current state (Equation 6)
        auto f0 = ComputeForces(state);

        // for (size_t i = 0; i < f0.size(); ++i) {
        //     std::cout << "Force on particle " << i << ": " << f0[i].x << ", " << f0[i].y << ", " << f0[i].z << std::endl;
        // }

        auto df_dx = ComputeForceJacobian(state);  // ∂f/∂x
        // std::cout << "Computed force Jacobian df/dx "  << df_dx.size() << std::endl;
        auto df_dv = ComputeDampingJacobian(state); // ∂f/∂v
        // std::cout << "Computed force Damping df/dx "  << df_dv.size() << std::endl;

        auto M = GetMassMatrix();

        Eigen::VectorXf f0_eigen = ToEigenVector(f0);
        Eigen::VectorXf v0_eigen = ToEigenVector(state.velocities);
        
        // Build the linear system (Equation 6):
        // (I - h*M^-1*∂f/∂v - h²*M^-1*∂f/∂x) Δv = h*M^-1(f0 + h*∂f/∂x*v0)
        
        // For symmetry, multiply through by M (Equation 15):
        // (M - h*∂f/∂v - h²*∂f/∂x) Δv = h(f0 + h*∂f/∂x*v0)
        
        Eigen::SparseMatrix<float> A = BuildSystemMatrix(
            df_dx, df_dv, dt);

        // std::cout << "Built system matrix A " << std::endl;
        // std::cout << A << std::endl;

        Eigen::VectorXf b = BuildRHS(
            f0_eigen, df_dx, v0_eigen, dt);

        // std::cout << "Built RHS vector b" << std::endl;
        // std::cout << b << std::endl;

        // std::vector<glm::vec3> b = BuildRHS(
        //     system, f0, df_dx, state.velocities, dt);
        
        // Solve using modified PCG (handles constraints)
        Eigen::VectorXf delta_v_eigen = pcg_solver.Solve(
            A, b, GetConstraints());
        
        std::vector<glm::vec3> delta_v = FromEigenVector(delta_v_eigen);

        // for (size_t i = 0; i < delta_v.size(); ++i) {
        //     std::cout << "Delta v for particle " << i << ": " 
        //               << delta_v[i].x << ", " 
        //               << delta_v[i].y << ", " 
        //               << delta_v[i].z << std::endl;
        // }

        return delta_v;
    }


    Eigen::SparseMatrix<float> BuildSystemMatrix(
        const Eigen::SparseMatrix<float>& df_dx,
        const Eigen::SparseMatrix<float>& df_dv,
        float dt) const {
        
        // A = M - h*∂f/∂v - h²*∂f/∂x

        // equation 6
        auto M = GetMassMatrix();
        for (int k=0; k<M.outerSize(); ++k) {
            for (Eigen::SparseMatrix<float>::InnerIterator it(M,k); it; ++it) {
                if (std::isnan(it.value())) {
                    throw std::runtime_error("NaN detected in mass matrix!");
                }
                // std::cout << "Mass matrix entry (" << it.row() << ", " << it.col() << "): " << it.value() << std::endl;
            }
        }
        return M - dt * df_dv - dt * dt * df_dx;
    }

    // NEW BuildRHS signature: Now uses Eigen::VectorXf for vector inputs and output.
    Eigen::VectorXf BuildRHS(
        const Eigen::VectorXf& f0_eigen, // f0 converted to Eigen
        const Eigen::SparseMatrix<float>& df_dx,
        const Eigen::VectorXf& v0_eigen, // v0 converted to Eigen
        float dt) const {
        
        // b = h(f0 + h*∂f/∂x*v0)
        
        // Eigen's sparse matrix-vector product is fast and correct: (∂f/∂x) * v0
        Eigen::VectorXf df_dx_v0 = df_dx * v0_eigen;
        
        // f0 + h*(∂f/∂x*v0)
        Eigen::VectorXf inside_bracket = f0_eigen + dt * df_dx_v0;
        
        // Multiply by h
        return dt * inside_bracket;
    }

    Eigen::VectorXf ToEigenVector(const std::vector<glm::vec3>& vec) const {
        size_t n = vec.size();
        Eigen::VectorXf result(3 * n);
        for (size_t i = 0; i < n; ++i) {
            result[3 * i + 0] = vec[i].x;
            result[3 * i + 1] = vec[i].y;
            result[3 * i + 2] = vec[i].z;
        }
        return result;
    }

    std::vector<glm::vec3> FromEigenVector(const Eigen::VectorXf& vec) const {
        size_t n = vec.size() / 3;
        std::vector<glm::vec3> result(n);
        for (size_t i = 0; i < n; ++i) {
            result[i] = glm::vec3(vec[3 * i + 0], vec[3 * i + 1], vec[3 * i + 2]);
        }
        return result;
    }

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

        for (auto& position : state.positions) {
            if (std::isnan(position.x) || std::isnan(position.y) || std::isnan(position.z)) {
                throw std::runtime_error("NaN detected in particle positions!");
            }

            // std::cout << "Particle position: " 
            //           << position.x << ", " 
            //           << position.y << ", " 
            //           << position.z << std::endl;
            
        }

        // for (const auto& triangle : triangles_) {
        //     std::cout << "Triangle vertices: " 
        //               << triangle.v0 << ", " 
        //               << triangle.v1 << ", " 
        //               << triangle.v2 << std::endl;
        //     std::cout << "Triangle area: " << triangle.area << std::endl;
        //     std::cout << "Triangle UVs: [" 
        //               << triangle.uv0.x << ", " << triangle.uv0.y << "; "
        //               << triangle.uv1.x << ", " << triangle.uv1.y << "; "
        //               << triangle.uv2.x << ", " << triangle.uv2.y << "]" << std::endl;
        //     std::cout << "Triangle UV inverse: [" 
        //               << triangle.uv_inv[0][0] << ", " << triangle.uv_inv[0][1] << "; "
        //               << triangle.uv_inv[1][0] << ", " << triangle.uv_inv[1][1] << "]" << std::endl;
        // }
        
        int middle = forces.size() / 2;

        // Gravity
        for (size_t i = 0; i < forces.size(); i++) {
            forces[i] += glm::vec3(0, -9.81f * masses_[i], 0);
        }

        // std::cout << "after gravity" << std::endl;
        // std::cout << "Force on particle " << middle << ": " << forces[middle].x << ", " << forces[middle].y << ", " << forces[middle].z << std::endl;

        if (wind_) {
            for (size_t i = 0; i < forces.size(); i++) {
                forces[i] += glm::vec3(0, -1.0f, 10.0f);
            }

            // std::cout << "after wind" << std::endl;
            // std::cout << "Force on particle " << middle << ": " << forces[middle].x << ", " << forces[middle].y << ", " << forces[middle].z << std::endl;
        }

        
        
        // Stretch forces (Section 4.2)
        for (const auto& tri : triangles_) {
            AddStretchForce(tri, state, forces);
        }
        
        // std::cout << "after stretch force" << std::endl;
        // std::cout << "Force on particle " << middle << ": " << forces[middle].x << ", " << forces[middle].y << ", " << forces[middle].z << std::endl;


        // Shear forces (Section 4.3)
        for (const auto& tri : triangles_) {
            AddShearForce(tri, state, forces);
        }
        
        // std::cout << "after shear force" << std::endl;
        // std::cout << "Force on particle " << middle << ": " << forces[middle].x << ", " << forces[middle].y << ", " << forces[middle].z << std::endl;

        // Bend forces (Section 4.3) edges shared by two triangles
        for (size_t i = 0; i < bend_springs_.size(); i++) {
            AddBendForce(bend_springs_[i], state, forces);
        }

        // std::cout << "after bend force" << std::endl;
        // std::cout << "Force on particle " << middle << ": " << forces[middle].x << ", " << forces[middle].y << ", " << forces[middle].z << std::endl;
        
        // Damping (Section 4.5)
        AddDampingForces(state, forces);

        // std::cout << "after damping" << std::endl;
        // std::cout << "Force on particle " << middle << ": " << forces[middle].x << ", " << forces[middle].y << ", " << forces[middle].z << std::endl;
        
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

    void AddMass(int idx, float mass) {
        if (idx >= masses_.size()) {
            masses_.resize(idx + 1, 0.0f);
        }
        masses_[idx] += mass;
    }
    
    void AddTriangle(int v0, int v1, int v2, 
                    glm::vec2 uv0, glm::vec2 uv1, glm::vec2 uv2) {
        Triangle tri;
        tri.v0 = v0;
        tri.v1 = v1;
        tri.v2 = v2;
        tri.uv0 = uv0;
        tri.uv1 = uv1;
        tri.uv2 = uv2;
        
        // Compute area in UV space
        glm::vec2 duv1 = uv1 - uv0;
        glm::vec2 duv2 = uv2 - uv0;
        tri.area = 0.5f * std::abs(duv1.x * duv2.y - duv1.y * duv2.x);
        
        // Precompute UV inverse (Equation 9)
        float det = duv1.x * duv2.y - duv1.y * duv2.x;
        if (std::abs(det) > 1e-6f) {
            tri.uv_inv = glm::mat2(
                duv2.y / det,  -duv2.x / det,   // First column
                -duv1.y / det,  duv1.x / det    // Second column
            );
        } else {
            // Degenerate triangle - use identity
            tri.uv_inv = glm::mat2(1.0f);
        }

        // std::cout << "Adding triangle: (" << v0 << ", " << v1 << ", " << v2 << ")" << std::endl;
        // std::cout << "with area: " << tri.area << std::endl;
        // std::cout << "and UVs: [" 
        //           << uv0.x << ", " << uv0.y << "; "
        //           << uv1.x << ", " << uv1.y << "; "
        //           << uv2.x << ", " << uv2.y << "]" << std::endl;
        // std::cout << "and UV inverse: [" 
        //           << tri.uv_inv[0][0] << ", " << tri.uv_inv[0][1] << "; "
        //           << tri.uv_inv[1][0] << ", " << tri.uv_inv[1][1] << "]" << std::endl;

        
        triangles_.push_back(tri);
    }
    
    void AddSpring(int i, int j, float stiffness, float rest_length) {
        structural_springs_.push_back({i, j, rest_length, stiffness});
    }
    
    void FixMass(int idx) {
        fixed_particles_.push_back(idx);
    }

    void Blow() {
        wind_ = !wind_;
    }

    Eigen::SparseMatrix<float> GetMassMatrix() const {
        int N = masses_.size();
        int size = 3 * N;
        std::vector<Eigen::Triplet<float>> triplets;
        
        // should be a diagonal 3n x 3n matrix 
        for (int i = 0; i < N; ++i) {
            // mass mi for x, y, z degrees of freedom
            triplets.emplace_back(3 * i, 3 * i, masses_[i]);
            triplets.emplace_back(3 * i + 1, 3 * i + 1, masses_[i]);
            triplets.emplace_back(3 * i + 2, 3 * i + 2, masses_[i]);
        }

        Eigen::SparseMatrix<float> M(size, size);
        M.setFromTriplets(triplets.begin(), triplets.end());
        return M;
    }
    
    std::vector<Constraint> GetConstraints() const {
        std::vector<Constraint> constraints;
        for (int idx : fixed_particles_) {
            Constraint c;
            c.particle_index = idx;
            c.dof = 0; // Fully constrained (0 degrees of freedom remain)
            c.z = glm::vec3(0.0f); // Desired velocity change (delta_v) is zero.
            constraints.push_back(c);
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

        // std::cout << "Computing stretch force for triangle (" 
        //           << tri.v0 << ", " << tri.v1 << ", " << tri.v2 << ")" << std::endl;
        // std::cout << "Positions: x0 = (" 
        //           << state.positions[tri.v0].x << ", " << state.positions[tri.v0].y << ", " << state.positions[tri.v0].z << "), "
        //           << "x1 = (" << state.positions[tri.v1].x << ", " << state.positions[tri.v1].y << ", " << state.positions[tri.v1].z << "), "
        //           << "x2 = (" << state.positions[tri.v2].x << ", " << state.positions[tri.v2].y << ", " << state.positions[tri.v2].z << ")" << std::endl;

        // for (const auto& triangle : triangles_) {
        //     std::cout << "Triangle vertices: " 
        //               << triangle.v0 << ", " 
        //               << triangle.v1 << ", " 
        //               << triangle.v2 << std::endl;
        //     std::cout << "Triangle area: " << triangle.area << std::endl;
        //     std::cout << "Triangle UVs: [" 
        //               << triangle.uv0.x << ", " << triangle.uv0.y << "; "
        //               << triangle.uv1.x << ", " << triangle.uv1.y << "; "
        //               << triangle.uv2.x << ", " << triangle.uv2.y << "]" << std::endl;
        //     std::cout << "Triangle UV inverse: [" 
        //               << triangle.uv_inv[0][0] << ", " << triangle.uv_inv[0][1] << "; "
        //               << triangle.uv_inv[1][0] << ", " << triangle.uv_inv[1][1] << "]" << std::endl;
        // }
        
        glm::vec3 w_u = x1 * tri.uv_inv[0][0] + x2 * tri.uv_inv[1][0];
        glm::vec3 w_v = x1 * tri.uv_inv[0][1] + x2 * tri.uv_inv[1][1];

        // std::cout << "w_u: (" << w_u.x << ", " << w_u.y << ", " << w_u.z << ")" << std::endl;
        // std::cout << "w_v: (" << w_v.x << ", " << w_v.y << ", " << w_v.z << ")" << std::endl;

        float len_u = glm::length(w_u);
        float len_v = glm::length(w_v);

        float C_u = tri.area * (len_u - 1.0f);  
        float C_v = tri.area * (len_v - 1.0f);

        // std::cout << "C_u: " << C_u << ", C_v: " << C_v << std::endl;
        
        // Force from Equation (7): f_i = -k * ∂C/∂x_i * C(x)
        // Need to compute ∂C/∂x for each vertex
        // ... (detailed implementation)

        glm::vec3 dC_u_dx0, dC_u_dx1, dC_u_dx2;

        if (len_u > 1e-6f) {
        // Normalized w_u (direction vector)
            glm::vec3 w_u_normalized = w_u / len_u;
            
            // ∂w_u/∂x coefficients (from Step 2)
            float dwu_dx0_coeff = -tri.uv_inv[0][0] - tri.uv_inv[1][0];
            float dwu_dx1_coeff = tri.uv_inv[0][0];
            float dwu_dx2_coeff = tri.uv_inv[1][0];
            
            // ∂C_u/∂x_i = area * (w_u / ||w_u||) * (∂w_u/∂x_i coefficient)
            dC_u_dx0 = tri.area * w_u_normalized * dwu_dx0_coeff;
            dC_u_dx1 = tri.area * w_u_normalized * dwu_dx1_coeff;
            dC_u_dx2 = tri.area * w_u_normalized * dwu_dx2_coeff;
        } else {
            // Degenerate case: w_u has zero length
            dC_u_dx0 = dC_u_dx1 = dC_u_dx2 = glm::vec3(0);
        }

        glm::vec3 dC_v_dx0, dC_v_dx1, dC_v_dx2;
    
        if (len_v > 1e-6f) {
            // Normalized w_v (direction vector)
            glm::vec3 w_v_normalized = w_v / len_v;
            
            // ∂w_v/∂x coefficients (from Step 2)
            float dwv_dx0_coeff = -tri.uv_inv[0][1] - tri.uv_inv[1][1];
            float dwv_dx1_coeff = tri.uv_inv[0][1];
            float dwv_dx2_coeff = tri.uv_inv[1][1];
            
            // ∂C_v/∂x_i = area * (w_v / ||w_v||) * (∂w_v/∂x_i coefficient)
            dC_v_dx0 = tri.area * w_v_normalized * dwv_dx0_coeff;
            dC_v_dx1 = tri.area * w_v_normalized * dwv_dx1_coeff;
            dC_v_dx2 = tri.area * w_v_normalized * dwv_dx2_coeff;
        } else {
            // Degenerate case: w_v has zero length
            dC_v_dx0 = dC_v_dx1 = dC_v_dx2 = glm::vec3(0);
        }

        float k = stretch_stiffness_;

        // std::cout << "Stretch force contributions for triangle (" 
        //           << tri.v0 << ", " << tri.v1 << ", " << tri.v2 << "): "
        //           << "dC_u_dx0 = (" << dC_u_dx0.x << ", " << dC_u_dx0.y << ", " << dC_u_dx0.z << "), "
        //           << "C_u = " << C_u << ", C_v = " << C_v << std::endl;
    
        // Each vertex gets force from both C_u and C_v
        forces[tri.v0] += -k * (dC_u_dx0 * C_u + dC_v_dx0 * C_v);
        forces[tri.v1] += -k * (dC_u_dx1 * C_u + dC_v_dx1 * C_v);
        forces[tri.v2] += -k * (dC_u_dx2 * C_u + dC_v_dx2 * C_v);
        

    }
    
    void AddShearForce(const Triangle& tri,
                      const ParticleState& state,
                      std::vector<glm::vec3>& forces) const {
        // C(x) = a * w_u^T * w_v (Section 4.3)
        // ... implementation
        glm::vec3 x1 = state.positions[tri.v1] - state.positions[tri.v0];
        glm::vec3 x2 = state.positions[tri.v2] - state.positions[tri.v0];

        glm::vec3 w_u = x1 * tri.uv_inv[0][0] + x2 * tri.uv_inv[1][0];
        glm::vec3 w_v = x1 * tri.uv_inv[0][1] + x2 * tri.uv_inv[1][1];

        float C = tri.area * glm::dot(w_u, w_v);

        glm::vec3 dC_dx0 = tri.area * (
            w_v * (-tri.uv_inv[0][0] - tri.uv_inv[1][0]) +
            w_u * (-tri.uv_inv[0][1] - tri.uv_inv[1][1])
        );
        
        glm::vec3 dC_dx1 = tri.area * (
            w_v * tri.uv_inv[0][0] +
            w_u * tri.uv_inv[0][1]
        );
        
        glm::vec3 dC_dx2 = tri.area * (
            w_v * tri.uv_inv[1][0] +
            w_u * tri.uv_inv[1][1]
        );
        
        // Force: f_i = -k * ∂C/∂x_i * C(x)
        float k = shear_stiffness_;
        
        forces[tri.v0] += -k * dC_dx0 * C;
        forces[tri.v1] += -k * dC_dx1 * C;
        forces[tri.v2] += -k * dC_dx2 * C;
    }
    
    void AddBendForce(const Spring& spring,
                     const ParticleState& state,
                     std::vector<glm::vec3>& forces) const {
        // Angle between adjacent triangles (Section 4.3)
        // ... implementation

        glm::vec3 d = state.positions[spring.i] - state.positions[spring.j];
        float length = glm::length(d);
        
        if (length < 1e-6f) return;
        
        // Bend force opposes deviation from rest angle
        // Using spring model: F = -k * (L - L0) * direction
        glm::vec3 force = -spring.stiffness * (length - spring.rest_length) * d / length;
        
        forces[spring.i] += force;
        forces[spring.j] -= force;
    }
    
    void AddDampingForces(const ParticleState& state,
                         std::vector<glm::vec3>& forces) const {
        // Equation (11): d = -k_d * ∂C/∂x * Ċ(x)
        // ... implementation

        for (const auto& tri : triangles_) {
            AddStretchDamping(tri, state, forces);
        }
        
        // Also add simple velocity damping
        for (size_t i = 0; i < state.velocities.size(); i++) {
            forces[i] += -damping_stiffness_ * state.velocities[i];
        }
    }

    void AddStretchDamping(const Triangle& tri,
                          const ParticleState& state,
                          std::vector<glm::vec3>& forces) const {
        
        // Compute deformation gradient
        glm::vec3 x1 = state.positions[tri.v1] - state.positions[tri.v0];
        glm::vec3 x2 = state.positions[tri.v2] - state.positions[tri.v0];
        
        glm::vec3 w_u = x1 * tri.uv_inv[0][0] + x2 * tri.uv_inv[1][0];
        glm::vec3 w_v = x1 * tri.uv_inv[0][1] + x2 * tri.uv_inv[1][1];
        
        float len_u = glm::length(w_u);
        float len_v = glm::length(w_v);
        
        if (len_u < 1e-6f || len_v < 1e-6f) return;
        
        // Compute velocity gradient
        glm::vec3 v1 = state.velocities[tri.v1] - state.velocities[tri.v0];
        glm::vec3 v2 = state.velocities[tri.v2] - state.velocities[tri.v0];
        
        glm::vec3 w_u_dot = v1 * tri.uv_inv[0][0] + v2 * tri.uv_inv[1][0];
        glm::vec3 w_v_dot = v1 * tri.uv_inv[0][1] + v2 * tri.uv_inv[1][1];
        
        // Ċ(x) = d/dt[||w_u|| - 1, ||w_v|| - 1]
        float C_dot_u = tri.area * glm::dot(w_u, w_u_dot) / len_u;
        float C_dot_v = tri.area * glm::dot(w_v, w_v_dot) / len_v;
        
        // ∂C/∂x (same as in AddStretchForce)
        glm::vec3 w_u_norm = w_u / len_u;
        glm::vec3 w_v_norm = w_v / len_v;
        
        glm::vec3 dC_u_dx0 = tri.area * w_u_norm * (-tri.uv_inv[0][0] - tri.uv_inv[1][0]);
        glm::vec3 dC_u_dx1 = tri.area * w_u_norm * tri.uv_inv[0][0];
        glm::vec3 dC_u_dx2 = tri.area * w_u_norm * tri.uv_inv[1][0];
        
        glm::vec3 dC_v_dx0 = tri.area * w_v_norm * (-tri.uv_inv[0][1] - tri.uv_inv[1][1]);
        glm::vec3 dC_v_dx1 = tri.area * w_v_norm * tri.uv_inv[0][1];
        glm::vec3 dC_v_dx2 = tri.area * w_v_norm * tri.uv_inv[1][1];
        
        // Damping force: d = -k_d * ∂C/∂x * Ċ(x) (Equation 11)
        float k_d = damping_stiffness_ * 0.1f;  // Smaller coefficient for damping
        
        forces[tri.v0] += -k_d * (dC_u_dx0 * C_dot_u + dC_v_dx0 * C_dot_v);
        forces[tri.v1] += -k_d * (dC_u_dx1 * C_dot_u + dC_v_dx1 * C_dot_v);
        forces[tri.v2] += -k_d * (dC_u_dx2 * C_dot_u + dC_v_dx2 * C_dot_v);
    }
    
    void AddStretchJacobian(const Triangle& tri,
                           const ParticleState& state,
                           std::vector<Eigen::Triplet<float>>& triplets) const {
        // equation 8
        // Compute current deformation
        glm::vec3 x1 = state.positions[tri.v1] - state.positions[tri.v0];
        glm::vec3 x2 = state.positions[tri.v2] - state.positions[tri.v0];
        
        glm::vec3 w_u = x1 * tri.uv_inv[0][0] + x2 * tri.uv_inv[1][0];
        glm::vec3 w_v = x1 * tri.uv_inv[0][1] + x2 * tri.uv_inv[1][1];
        
        float len_u = glm::length(w_u);
        float len_v = glm::length(w_v);
        
        if (len_u < 1e-6f || len_v < 1e-6f) return;
        
        // Condition values
        float C_u = tri.area * (len_u - 1.0f);
        float C_v = tri.area * (len_v - 1.0f);
        
        int vertices[3] = {tri.v0, tri.v1, tri.v2};
        glm::vec3 dC_u[3], dC_v[3];
        
        // Compute gradients
        glm::vec3 w_u_norm = w_u / len_u;
        dC_u[0] = tri.area * w_u_norm * (-tri.uv_inv[0][0] - tri.uv_inv[1][0]);
        dC_u[1] = tri.area * w_u_norm * tri.uv_inv[0][0];
        dC_u[2] = tri.area * w_u_norm * tri.uv_inv[1][0];
        
        glm::vec3 w_v_norm = w_v / len_v;
        dC_v[0] = tri.area * w_v_norm * (-tri.uv_inv[0][1] - tri.uv_inv[1][1]);
        dC_v[1] = tri.area * w_v_norm * tri.uv_inv[0][1];
        dC_v[2] = tri.area * w_v_norm * tri.uv_inv[1][1];
        
        float k = stretch_stiffness_;
        
        // K_ij = -k(∂C/∂x_i * (∂C/∂x_j)^T + ∂²C/∂x_i∂x_j * C(x))
        // First term (outer product):
        for (int a = 0; a < 3; a++) {
            for (int b = 0; b < 3; b++) {
                int i = vertices[a];
                int j = vertices[b];
                
                // K_ij = -k * (∂C_u/∂x_i ⊗ ∂C_u/∂x_j + ∂C_v/∂x_i ⊗ ∂C_v/∂x_j)
                glm::mat3 K_ij_outer = -k * (glm::outerProduct(dC_u[a], dC_u[b]) +
                                             glm::outerProduct(dC_v[a], dC_v[b]));
                
                // Second term (Hessian of C):
                // ∂²C_u/∂x_i∂x_j = a/||w_u|| * (I - w_u⊗w_u/||w_u||²) * ∂w_u/∂x_i * ∂w_u/∂x_j^T
                // This is complex, so we use a common approximation: include only when i == j
                glm::mat3 K_ij_hessian(0.0f);
                if (a == b && std::abs(C_u) > 1e-6f) {
                    // Simplified Hessian contribution
                    float coeff_u = tri.area / (len_u * len_u * len_u);
                    glm::mat3 I = glm::mat3(1.0f);
                    glm::mat3 wu_outer = glm::outerProduct(w_u, w_u);
                    
                    float scale_u = tri.uv_inv[0][0] + tri.uv_inv[1][0];
                    if (a == 1) scale_u = tri.uv_inv[0][0];
                    if (a == 2) scale_u = tri.uv_inv[1][0];
                    
                    K_ij_hessian += -k * C_u * coeff_u * (I - wu_outer / (len_u * len_u)) * scale_u * scale_u;
                }
                
                if (a == b && std::abs(C_v) > 1e-6f) {
                    float coeff_v = tri.area / (len_v * len_v * len_v);
                    glm::mat3 I = glm::mat3(1.0f);
                    glm::mat3 wv_outer = glm::outerProduct(w_v, w_v);
                    
                    float scale_v = tri.uv_inv[0][1] + tri.uv_inv[1][1];
                    if (a == 1) scale_v = tri.uv_inv[0][1];
                    if (a == 2) scale_v = tri.uv_inv[1][1];
                    
                    K_ij_hessian += -k * C_v * coeff_v * (I - wv_outer / (len_v * len_v)) * scale_v * scale_v;
                }
                
                glm::mat3 K_ij = K_ij_outer + K_ij_hessian;
                
                // Add to sparse matrix (note: glm is column-major)
                for (int row = 0; row < 3; row++) {
                    for (int col = 0; col < 3; col++) {
                        triplets.push_back(Eigen::Triplet<float>(
                            3*i + row, 3*j + col, K_ij[col][row]));
                    }
                }
            }
        }
    }
    
    void AddShearJacobian(const Triangle& tri,
                         const ParticleState& state,
                         std::vector<Eigen::Triplet<float>>& triplets) const {
        // Equation (8) applied to shear condition
        // ... implementation
        glm::vec3 x1 = state.positions[tri.v1] - state.positions[tri.v0];
        glm::vec3 x2 = state.positions[tri.v2] - state.positions[tri.v0];
        
        glm::vec3 w_u = x1 * tri.uv_inv[0][0] + x2 * tri.uv_inv[1][0];
        glm::vec3 w_v = x1 * tri.uv_inv[0][1] + x2 * tri.uv_inv[1][1];
        
        // Condition: C = a * w_u^T * w_v
        float C = tri.area * glm::dot(w_u, w_v);
        
        int vertices[3] = {tri.v0, tri.v1, tri.v2};
        glm::vec3 dC_dx[3];
        
        // ∂C/∂x
        dC_dx[0] = tri.area * (
            w_v * (-tri.uv_inv[0][0] - tri.uv_inv[1][0]) +
            w_u * (-tri.uv_inv[0][1] - tri.uv_inv[1][1])
        );
        
        dC_dx[1] = tri.area * (
            w_v * tri.uv_inv[0][0] +
            w_u * tri.uv_inv[0][1]
        );
        
        dC_dx[2] = tri.area * (
            w_v * tri.uv_inv[1][0] +
            w_u * tri.uv_inv[1][1]
        );
        
        float k = shear_stiffness_;
        
        // K_ij = -k * (∂C/∂x_i ⊗ ∂C/∂x_j + ∂²C/∂x_i∂x_j * C)
        for (int a = 0; a < 3; a++) {
            for (int b = 0; b < 3; b++) {
                int i = vertices[a];
                int j = vertices[b];
                
                // First term: outer product
                glm::mat3 K_ij_outer = -k * glm::outerProduct(dC_dx[a], dC_dx[b]);
                
                // Second term: ∂²C/∂x_i∂x_j * C
                // ∂²C/∂x_i∂x_j = a * (∂w_u/∂x_i * ∂w_v/∂x_j^T + ∂w_v/∂x_i * ∂w_u/∂x_j^T)
                glm::mat3 K_ij_hessian(0.0f);
                
                // Get coefficients for w_u and w_v derivatives
                float du_i[3] = {
                    -tri.uv_inv[0][0] - tri.uv_inv[1][0],
                    tri.uv_inv[0][0],
                    tri.uv_inv[1][0]
                };
                float dv_i[3] = {
                    -tri.uv_inv[0][1] - tri.uv_inv[1][1],
                    tri.uv_inv[0][1],
                    tri.uv_inv[1][1]
                };
                
                // ∂w_u/∂x_i is just the coefficient
                // glm::vec3 dwu_dxi = glm::vec3(du_i[a]);
                // glm::vec3 dwv_dxi = glm::vec3(dv_i[a]);
                // glm::vec3 dwu_dxj = glm::vec3(du_i[b]);
                // glm::vec3 dwv_dxj = glm::vec3(dv_i[b]);
                
                // For a 3x3 matrix, this is actually scalar * I
                float hess_val = tri.area * (du_i[a] * dv_i[b] + dv_i[a] * du_i[b]);
                K_ij_hessian = -k * C * hess_val * glm::mat3(1.0f);
                
                glm::mat3 K_ij = K_ij_outer + K_ij_hessian;
                
                // Add to sparse matrix
                for (int row = 0; row < 3; row++) {
                    for (int col = 0; col < 3; col++) {
                        triplets.push_back(Eigen::Triplet<float>(
                            3*i + row, 3*j + col, K_ij[col][row]));
                    }
                }
            }
        }
    }
    
    void AddBendJacobian(const Spring& spring,
                        const ParticleState& state,
                        std::vector<Eigen::Triplet<float>>& triplets) const {
        // Equation (8) applied to bend condition
        // ... implementation
        // Same as AddSpringJacobian for bend springs
        glm::vec3 d = state.positions[spring.i] - state.positions[spring.j];
        float length = glm::length(d);
        
        if (length < 1e-6f) return;
        
        glm::vec3 n = d / length;
        float k = spring.stiffness;
        float r = spring.rest_length;
        
        // Jacobian of spring force
        // K = -k * [(1 - r/L) * I/L + (r/L - 1) * n⊗n]
        glm::mat3 K = -k * ((1.0f - r/length) * glm::mat3(1.0f) / length + 
                            (r/length - 1.0f) * glm::outerProduct(n, n));
        
        int i = spring.i;
        int j = spring.j;
        
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                float val = K[col][row];
                triplets.push_back(Eigen::Triplet<float>(3*i + row, 3*i + col, val));
                triplets.push_back(Eigen::Triplet<float>(3*j + row, 3*j + col, val));
                triplets.push_back(Eigen::Triplet<float>(3*i + row, 3*j + col, -val));
                triplets.push_back(Eigen::Triplet<float>(3*j + row, 3*i + col, -val));
            }
        }
    }
    
    void AddDampingJacobian(const Triangle& tri,
                           const ParticleState& state,
                           std::vector<Eigen::Triplet<float>>& triplets) const {
        // Equation (12) (simplified) applied to damping
        // ... implementation
        // ∂d_i/∂v_j = -k_d * ∂C/∂x_i * (∂C/∂x_j)^T
        // This is similar to the stretch Jacobian but with damping coefficient
        glm::vec3 x1 = state.positions[tri.v1] - state.positions[tri.v0];
        glm::vec3 x2 = state.positions[tri.v2] - state.positions[tri.v0];
        
        glm::vec3 w_u = x1 * tri.uv_inv[0][0] + x2 * tri.uv_inv[1][0];
        glm::vec3 w_v = x1 * tri.uv_inv[0][1] + x2 * tri.uv_inv[1][1];
        
        float len_u = glm::length(w_u);
        float len_v = glm::length(w_v);
        
        if (len_u < 1e-6f || len_v < 1e-6f) return;
        
        int vertices[3] = {tri.v0, tri.v1, tri.v2};
        glm::vec3 dC_u[3], dC_v[3];
        
        glm::vec3 w_u_norm = w_u / len_u;
        dC_u[0] = tri.area * w_u_norm * (-tri.uv_inv[0][0] - tri.uv_inv[1][0]);
        dC_u[1] = tri.area * w_u_norm * tri.uv_inv[0][0];
        dC_u[2] = tri.area * w_u_norm * tri.uv_inv[1][0];
        
        glm::vec3 w_v_norm = w_v / len_v;
        dC_v[0] = tri.area * w_v_norm * (-tri.uv_inv[0][1] - tri.uv_inv[1][1]);
        dC_v[1] = tri.area * w_v_norm * tri.uv_inv[0][1];
        dC_v[2] = tri.area * w_v_norm * tri.uv_inv[1][1];
        
        float k_d = damping_stiffness_ * 0.1f;
        
        // ∂d_i/∂v_j = -k_d * ∂C/∂x_i * (∂C/∂x_j)^T
        for (int a = 0; a < 3; a++) {
            for (int b = 0; b < 3; b++) {
                int i = vertices[a];
                int j = vertices[b];
                
                glm::mat3 D_ij = -k_d * (glm::outerProduct(dC_u[a], dC_u[b]) +
                                         glm::outerProduct(dC_v[a], dC_v[b]));
                
                for (int row = 0; row < 3; row++) {
                    for (int col = 0; col < 3; col++) {
                        triplets.push_back(Eigen::Triplet<float>(
                            3*i + row, 3*j + col, D_ij[col][row]));
                    }
                }
            }
        }
    }
};

}  // namespace GLOO

#endif