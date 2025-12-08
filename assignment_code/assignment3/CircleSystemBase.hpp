#ifndef CIRCLE_SYSTEM_BASE_H_
#define CIRCLE_SYSTEM_BASE_H_

#include "ParticleState.hpp"
#include "Eigen/Sparse"
#include "ModifiedPCG.hpp"
#include "ParticleSystemBase.hpp"
#include <iostream>

namespace GLOO {
    class CircleSystemBase : public ParticleSystemBase {
        public:
            virtual ~CircleSystemBase() {
                
            }
            
            ParticleState ComputeTimeDerivative(const ParticleState& state,
                float time) const override {
                ParticleState f;
                for (int i = 0; i < state.positions.size(); i++) {
                    glm::vec3 new_position = {-state.positions[i][1], state.positions[i][0], 0.0f};
                    // std::cout << "New Position: " << new_position[0] << ", " << new_position[1] << ", " << new_position[2] << std::endl;

                    f.positions.push_back(new_position);
                    f.velocities.push_back(state.velocities[i]);
                }
                return f;
            }

            // Minimal stubs so integrators (BackwardEuler, etc.) compile when
            // instantiated with CircleSystemBase. These return zero/empty data
            // sized according to the provided state.
            std::vector<glm::vec3> ComputeForces(const ParticleState& state) const {
                return std::vector<glm::vec3>(state.positions.size(), glm::vec3(0.0f));
            }

            Eigen::SparseMatrix<float> ComputeForceJacobian(const ParticleState& state) const {
                int n = static_cast<int>(state.positions.size());
                return Eigen::SparseMatrix<float>(3*n, 3*n);
            }

            Eigen::SparseMatrix<float> ComputeDampingJacobian(const ParticleState& state) const {
                int n = static_cast<int>(state.positions.size());
                return Eigen::SparseMatrix<float>(3*n, 3*n);
            }

            Eigen::SparseMatrix<float> GetInverseMassMatrix() const {
                return Eigen::SparseMatrix<float>(1, 1);
            }
            
            Eigen::SparseMatrix<float> GetMassMatrix() const {
                return Eigen::SparseMatrix<float>(1, 1);
            }

            std::vector<Constraint> GetConstraints() const {
                std::vector<Constraint> constraints;
                return constraints;
            }

            std::vector<glm::vec3> ComputeDeltaVelocity(
                const ParticleState& state,
                float dt, 
                const ModifiedPCG& pcg_solver
            ) const {
                std::cout << "ComputeDeltaVelocity called in CircleSystemBase" << std::endl;
                return std::vector<glm::vec3>(state.positions.size(), glm::vec3(0.0f));
            }





    };
}  // namespace GLOO

#endif
