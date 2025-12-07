// BackwardEulerIntegrator.hpp
#ifndef BACKWARD_EULER_INTEGRATOR_H_
#define BACKWARD_EULER_INTEGRATOR_H_

#include "IntegratorBase.hpp"
#include "ClothSystem.hpp"
#include "ModifiedPCG.hpp"
#include <Eigen/Sparse>

namespace GLOO {

template <class TSystem, class TState>
class BackwardEulerIntegrator : public IntegratorBase<TSystem, TState> {
private:
    ModifiedPCG pcg_solver_;
    
public:
    TState Integrate(const TSystem& system,
                     const TState& state,
                     float start_time,
                     float dt) const override {
        
        // Evaluate at current state (Equation 6)
        auto f0 = system.ComputeForces(state);
        auto df_dx = system.ComputeForceJacobian(state);  // ∂f/∂x
        auto df_dv = system.ComputeDampingJacobian(state); // ∂f/∂v
        auto M_inv = system.GetInverseMassMatrix();
        auto M = system.GetMassMatrix();
        
        // Build the linear system (Equation 6):
        // (I - h*M^-1*∂f/∂v - h²*M^-1*∂f/∂x) Δv = h*M^-1(f0 + h*∂f/∂x*v0)
        
        // For symmetry, multiply through by M (Equation 15):
        // (M - h*∂f/∂v - h²*∂f/∂x) Δv = h(f0 + h*∂f/∂x*v0)
        
        Eigen::SparseMatrix<float> A = BuildSystemMatrix(
            system, df_dx, df_dv, dt);
        std::vector<glm::vec3> b = BuildRHS(
            system, f0, df_dx, state.velocities, dt);
        
        // Solve using modified PCG (handles constraints)
        std::vector<glm::vec3> delta_v = pcg_solver_.Solve(
            A, b, system.GetConstraints());
        
        // Update state
        TState new_state = state;
        for (size_t i = 0; i < state.positions.size(); i++) {
            new_state.velocities[i] = state.velocities[i] + delta_v[i];
            new_state.positions[i] = state.positions[i] + 
                                      dt * new_state.velocities[i];
        }
        
        return new_state;
    }
    
private:
    Eigen::SparseMatrix<float> BuildSystemMatrix(
        const TSystem& system,
        const Eigen::SparseMatrix<float>& df_dx,
        const Eigen::SparseMatrix<float>& df_dv,
        float dt) const {
        
        // A = M - h*∂f/∂v - h²*∂f/∂x

        // equation 6
        auto M = system.GetMassMatrix();
        return M - dt * df_dv - dt * dt * df_dx;
    }
    
    std::vector<glm::vec3> BuildRHS(
        const TSystem& system,
        const std::vector<glm::vec3>& f0,
        const Eigen::SparseMatrix<float>& df_dx,
        const std::vector<glm::vec3>& v0,
        float dt) const {
        
        // b = h(f0 + h*∂f/∂x*v0)
        size_t n = f0.size();
        std::vector<glm::vec3> b(n, glm::vec3(0.0f));
        for (size_t i = 0; i < n; i++) {
            glm::vec3 df_dx_v0(0.0f);
            for (Eigen::SparseMatrix<float>::InnerIterator it(df_dx, 3*i); it; ++it) {
                int col = it.col();
                float value = it.value();
                df_dx_v0 += value * v0[col / 3]; // col/3 to get particle index
            }
            b[i] = dt * (f0[i] + dt * df_dx_v0);
        }
        return b;
    }
};

}  // namespace GLOO

#endif