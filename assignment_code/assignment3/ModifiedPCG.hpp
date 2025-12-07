// ModifiedPCG.hpp
#ifndef MODIFIED_PCG_H_
#define MODIFIED_PCG_H_

#include <Eigen/Sparse>
#include <glm/glm.hpp>
#include <vector>

namespace GLOO {

struct Constraint {
    int particle_index;
    int dof;  // degrees of freedom: 0 (fully constrained), 1, 2, or 3
    glm::vec3 p, q;  // constraint directions (if dof < 3)
    glm::vec3 z;     // desired velocity change in constrained directions
};
// ModifiedPCG.hpp - Alternative version using Eigen VectorXf

class ModifiedPCG {
private:
    float tolerance_ = 1e-4f;
    int max_iterations_ = 100;
    
public:
    // Change signature to work with Eigen vectors
    Eigen::VectorXf Solve(
        const Eigen::SparseMatrix<float>& A,
        const Eigen::VectorXf& b,
        const std::vector<Constraint>& constraints) const {
        
        int n = b.size() / 3;  // Number of particles
        Eigen::VectorXf delta_v(b.size());
        delta_v.setZero();
        
        // Line 2: Initialize with constraint values
        for (int i = 0; i < n; i++) {
            glm::vec3 z = GetConstraintValue(i, constraints);
            delta_v[3*i + 0] = z.x;
            delta_v[3*i + 1] = z.y;
            delta_v[3*i + 2] = z.z;
        }
        
        // Line 3: δ_0 = filter(b)^T P filter(b)
        Eigen::VectorXf filtered_b = Filter(b, constraints, n);
        Eigen::VectorXf precond_filtered_b = Precondition(filtered_b, A);
        float delta_0 = filtered_b.dot(precond_filtered_b);
        
        // Line 4: r = filter(b - A*Δv)
        Eigen::VectorXf r = Filter(b - A * delta_v, constraints, n);
        
        // Line 5: c = filter(P^-1 * r)
        Eigen::VectorXf c = Filter(Precondition(r, A), constraints, n);
        
        // Line 6: δ_new = r^T * c
        float delta_new = r.dot(c);
        
        // Line 7-15: Main CG loop
        for (int iter = 0; iter < max_iterations_; iter++) {
            if (delta_new < tolerance_ * tolerance_ * delta_0) {
                break;  // Converged
            }
            
            // Line 8: q = filter(A*c)
            Eigen::VectorXf q = Filter(A * c, constraints, n);
            
            // Line 9: α = δ_new / (c^T * q)
            float alpha = delta_new / c.dot(q);
            
            // Line 10: Δv = Δv + α*c
            delta_v += alpha * c;
            
            // Line 11: r = r - α*q
            r -= alpha * q;
            
            // Line 12: s = P^-1 * r
            Eigen::VectorXf s = Precondition(r, A);
            
            // Line 13-14: Update δ
            float delta_old = delta_new;
            delta_new = r.dot(s);
            
            // Line 15: c = filter(s + (δ_new/δ_old)*c)
            c = Filter(s + (delta_new / delta_old) * c, constraints, n);
        }
        
        return delta_v;
    }
    
private:
    // Filter operation - now works with Eigen vectors
    Eigen::VectorXf Filter(
        const Eigen::VectorXf& vec,
        const std::vector<Constraint>& constraints,
        int n_particles) const {
        
        Eigen::VectorXf result = vec;
        
        for (const auto& constraint : constraints) {
            int i = constraint.particle_index;
            glm::mat3 S = ComputeFilterMatrix(constraint);
            
            // Extract glm::vec3, apply filter, put back
            glm::vec3 v(vec[3*i + 0], vec[3*i + 1], vec[3*i + 2]);
            glm::vec3 filtered = S * v;
            
            result[3*i + 0] = filtered.x;
            result[3*i + 1] = filtered.y;
            result[3*i + 2] = filtered.z;
        }
        
        return result;
    }
    
    glm::mat3 ComputeFilterMatrix(const Constraint& c) const {
        // Same as before...
        if (c.dof == 3) {
            return glm::mat3(1.0f);
        } else if (c.dof == 2) {
            return glm::mat3(1.0f) - glm::outerProduct(c.p, c.p);
        } else if (c.dof == 1) {
            return glm::mat3(1.0f) - 
                   glm::outerProduct(c.p, c.p) - 
                   glm::outerProduct(c.q, c.q);
        } else {
            return glm::mat3(0.0f);
        }
    }
    
    glm::vec3 GetConstraintValue(
        int i, const std::vector<Constraint>& constraints) const {
        
        for (const auto& c : constraints) {
            if (c.particle_index == i) {
                return c.z;
            }
        }
        return glm::vec3(0.0f);
    }
    
    // Simple diagonal preconditioner
    Eigen::VectorXf Precondition(
        const Eigen::VectorXf& vec,
        const Eigen::SparseMatrix<float>& A) const {
        
        Eigen::VectorXf result = vec;
        int n = vec.size() / 3;
        
        for (int i = 0; i < n; i++) {
            // Get diagonal element (assuming same for x, y, z)
            float diag = A.coeff(3*i, 3*i);
            if (diag > 1e-10f) {  // Avoid division by zero
                result[3*i + 0] /= diag;
                result[3*i + 1] /= diag;
                result[3*i + 2] /= diag;
            }
        }
        
        return result;
    }
};

}  // namespace GLOO

#endif