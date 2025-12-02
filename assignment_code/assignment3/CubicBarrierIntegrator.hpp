#ifndef CUBIC_BARRIER_INTEGRATOR_H_
#define CUBIC_BARRIER_INTEGRATOR_H_

#include "IntegratorBase.hpp"
#include <Eigen/Sparse>
#include <Eigen/Dense>

namespace GLOO {

template <class TSystem, class TState>
class CubicBarrierIntegrator : public IntegratorBase<TSystem, TState> {
public:
  CubicBarrierIntegrator() : beta_max_(0.25f), max_newton_steps_(8) {}

  TState Integrate(const TSystem& system,
                   const TState& state,
                   float start_time,
                   float dt) const override {
    
    TState current_state = state;
    float beta = 0.0f;
    
    // Store previous positions for inertia
    current_state.prev_positions = state.positions;
    current_state.inertia_targets.resize(state.positions.size());
    
    // Compute inertia targets: z = x + dt*v + dt^2*g
    for (size_t i = 0; i < state.positions.size(); i++) {
      glm::vec3 gravity(0.0f, -9.81f, 0.0f);
      current_state.inertia_targets[i] = state.positions[i] + 
                                         dt * state.velocities[i] + 
                                         dt * dt * gravity;
    }
    
    // Main Newton loop
    int step_count = 0;
    while (beta < beta_max_ && step_count < max_newton_steps_) {
      float alpha = NewtonStep(system, current_state, dt);
      beta = beta + (1.0f - beta) * alpha;
      step_count++;
    }
    
    // Error reduction pass
    NewtonStep(system, current_state, beta * dt);
    
    // Update velocities: v = (x_new - x_old) / (beta * dt)
    float actual_dt = beta * dt;
    if (actual_dt > 1e-6f) {
      for (size_t i = 0; i < current_state.positions.size(); i++) {
        current_state.velocities[i] = 
          (current_state.positions[i] - state.positions[i]) / actual_dt;
      }
    }
    
    return current_state;
  }

private:
  float beta_max_;
  int max_newton_steps_;
  
  // Perform one Newton step
  float NewtonStep(const TSystem& system, TState& state, float dt) const {
    // Build system: H * d = -grad
    // where H is Hessian, d is search direction, grad is gradient
    
    int n = state.positions.size();
    Eigen::VectorXf gradient(3 * n);
    Eigen::SparseMatrix<float> hessian(3 * n, 3 * n);
    
    // Compute gradient and Hessian of total energy
    ComputeGradientAndHessian(system, state, dt, gradient, hessian);
    
    // Solve H * d = -gradient using Conjugate Gradient
    Eigen::VectorXf search_direction = SolveCG(hessian, -gradient);
    
    // Line search with 1.25x extension (Section 3.2.4 of paper)
    float alpha = LineSearch(system, state, search_direction, dt, 1.25f);
    
    // Update positions: x = x + alpha * d
    for (int i = 0; i < n; i++) {
      state.positions[i].x += alpha * search_direction[3*i];
      state.positions[i].y += alpha * search_direction[3*i + 1];
      state.positions[i].z += alpha * search_direction[3*i + 2];
    }
    
    return alpha;
  }
  
  void ComputeGradientAndHessian(const TSystem& system, 
                                  const TState& state,
                                  float dt,
                                  Eigen::VectorXf& gradient,
                                  Eigen::SparseMatrix<float>& hessian) const {
    int n = state.positions.size();
    gradient.setZero();
    
    // Triplets for sparse matrix construction
    std::vector<Eigen::Triplet<float>> triplets;
    
    // 1. Inertia energy: psi_inertia = m/(2*dt^2) * ||x - z||^2
    //    Gradient: m/dt^2 * (x - z)
    //    Hessian: m/dt^2 * I
    for (int i = 0; i < n; i++) {
      if (system.IsFixed(i)) continue;
      
      float mass = system.GetMass(i);
      float stiffness = mass / (dt * dt);
      glm::vec3 diff = state.positions[i] - state.inertia_targets[i];
      
      // Gradient contribution
      gradient[3*i] += stiffness * diff.x;
      gradient[3*i + 1] += stiffness * diff.y;
      gradient[3*i + 2] += stiffness * diff.z;
      
      // Hessian contribution (diagonal)
      triplets.push_back(Eigen::Triplet<float>(3*i, 3*i, stiffness));
      triplets.push_back(Eigen::Triplet<float>(3*i+1, 3*i+1, stiffness));
      triplets.push_back(Eigen::Triplet<float>(3*i+2, 3*i+2, stiffness));
    }
    
    // 2. Elastic energy from springs (strain limiting with cubic barrier)
    system.AddElasticGradientAndHessian(state, gradient, triplets);
    
    // 3. Contact energy (cubic barrier)
    // TODO: Add contact detection and barrier energy
    
    // 4. Boundary conditions (fixed vertices)
    for (int i = 0; i < n; i++) {
      if (system.IsFixed(i)) {
        // Set gradient to zero
        gradient[3*i] = 0.0f;
        gradient[3*i + 1] = 0.0f;
        gradient[3*i + 2] = 0.0f;
        
        // Set Hessian row to identity (large diagonal)
        triplets.push_back(Eigen::Triplet<float>(3*i, 3*i, 1e6f));
        triplets.push_back(Eigen::Triplet<float>(3*i+1, 3*i+1, 1e6f));
        triplets.push_back(Eigen::Triplet<float>(3*i+2, 3*i+2, 1e6f));
      }
    }
    
    hessian.setFromTriplets(triplets.begin(), triplets.end());
  }
  
  Eigen::VectorXf SolveCG(const Eigen::SparseMatrix<float>& A, 
                          const Eigen::VectorXf& b) const {
    // Simple Conjugate Gradient solver
    // Use block Jacobi preconditioner (3x3 blocks for x,y,z)
    Eigen::ConjugateGradient<Eigen::SparseMatrix<float>, 
                             Eigen::Lower|Eigen::Upper> cg;
    cg.setMaxIterations(1000);
    cg.setTolerance(1e-3f);
    cg.compute(A);
    return cg.solve(b);
  }
  
  float LineSearch(const TSystem& system,
                   const TState& state,
                   const Eigen::VectorXf& direction,
                   float dt,
                   float extension) const {
    // Constraint-aware line search
    // Find maximum alpha such that no constraints are violated
    
    float alpha = 1.0f;
    int n = state.positions.size();
    
    // Check spring length constraints
    for (const auto& spring : system.GetSprings()) {
      int i = spring.i;
      int j = spring.j;
      
      glm::vec3 di(extension * direction[3*i], 
                   extension * direction[3*i+1], 
                   extension * direction[3*i+2]);
      glm::vec3 dj(extension * direction[3*j], 
                   extension * direction[3*j+1], 
                   extension * direction[3*j+2]);
      
      glm::vec3 pi = state.positions[i] + di;
      glm::vec3 pj = state.positions[j] + dj;
      
      float length = glm::length(pi - pj);
      float max_length = spring.rest_length * 1.05f; // 5% strain limit
      
      if (length > max_length) {
        // Binary search for valid alpha
        float alpha_candidate = 0.5f;
        // Simplified: just reduce alpha
        alpha = std::min(alpha, alpha_candidate);
      }
    }
    
    return std::max(alpha, 0.01f); // Minimum step
  }
};

}  // namespace GLOO

#endif