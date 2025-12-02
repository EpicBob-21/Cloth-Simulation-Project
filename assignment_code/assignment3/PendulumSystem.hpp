#ifndef PENDULUM_SYSTEM_H_
#define PENDULUM_SYSTEM_H_

#include "ParticleState.hpp"
#include "ParticleSystemBase.hpp"
#include <map>
#include "Eigen/Sparse"
#include <set>



namespace GLOO {
    class PendulumSystem : public ParticleSystemBase {
        public:
            struct Spring {
                int i, j;
                float stiffness;
                float rest_length;
            };

            virtual ~PendulumSystem() {

            }
            
            ParticleState ComputeTimeDerivative(const ParticleState& state,
                float time) const;

            void AddSpring(int i, int j, float k, float r) {
                // https://stackoverflow.com/questions/15016646/using-pairint-int-as-key-for-map
                ks_rs_[std::make_pair(i, j)] = std::make_pair(k, r);
            }

            void AddMass(int i, float m, float d) {
                masses_[i] = m;
                dampings_[i] = d;
            }

            void FixMass(int i) {
                fixeds_.insert(i);
            }

            void Blow() {
                wind_ = !wind_;
            }

            std::vector<Spring> GetSprings() const {
                std::vector<Spring> springs;
                for (const auto& [key, value] : ks_rs_) {
                springs.push_back({key.first, key.second, value.first, value.second});
                }
                return springs;
            }
            
            bool IsFixed(int index) const {
                return fixeds_.count(index) > 0;
            }
            
            float GetMass(int index) const {
                return masses_.at(index);
            }

            void AddElasticGradientAndHessian(const ParticleState& state,
                                                Eigen::VectorXf& gradient,
                                                std::vector<Eigen::Triplet<float>>& triplets) const {
                // For each spring, add strain limiting energy with cubic barrier
                for (const auto& [key, value] : ks_rs_) {
                int i = key.first;
                int j = key.second;
                float k_spring = value.first;
                float rest_length = value.second;
                
                glm::vec3 edge = state.positions[i] - state.positions[j];
                float current_length = glm::length(edge);
                
                if (current_length < 1e-6f) continue; // Avoid division by zero
                
                glm::vec3 direction = edge / current_length;
                
                // Strain: sigma = current_length / rest_length
                float sigma = current_length / rest_length;
                
                // Cubic barrier parameters section 3.6
                float tau = 0.025f;  // he sets this in the paper
                float epsilon_hat = 0.025f;  // tau is same as epsilon_hat in paper
                float g = 1.0f + tau + epsilon_hat - sigma;  // Geometric gap
                
                if (g > epsilon_hat) continue; // not within gap
                
                // (9), stiffness of strain limiting
                float mass_face = (masses_.at(i) + masses_.at(j)) / 2.0f;

                // TODO add hessian part
                float kappa = mass_face / (g * g + 1e-6f);
                
                // cubic barrier energy, (1)
                // psi = -2*kappa/(3*epsilon_hat) * (g - epsilon_hat)^3
                float g_diff = g - epsilon_hat;
                float energy_factor = -2.0f * kappa / (3.0f * epsilon_hat);
                
                // Gradient of cubic barrier w.r.t. g
                // d(psi)/dg = -2*kappa/epsilon_hat * (g - epsilon_hat)^2
                float grad_g = -2.0f * kappa / epsilon_hat * g_diff * g_diff;
                
                // Hessian of cubic barrier w.r.t. g:
                // d^2(psi)/dg^2 = -4*kappa/epsilon_hat * (g - epsilon_hat)
                float hess_g = -4.0f * kappa / epsilon_hat * g_diff;
                
                // Chain rule: g = 1 + tau + epsilon_hat - sigma
                //             sigma = current_length / rest_length
                // So: dg/dx = -1/rest_length * d(current_length)/dx
                //     d(current_length)/dx = direction for vertex i, -direction for vertex j
                
                glm::vec3 grad_i = -(grad_g / rest_length) * direction;
                glm::vec3 grad_j = -grad_i;
                
                // Add to gradient
                if (!fixeds_.count(i)) {
                    gradient[3*i] += grad_i.x;
                    gradient[3*i + 1] += grad_i.y;
                    gradient[3*i + 2] += grad_i.z;
                }
                
                if (!fixeds_.count(j)) {
                    gradient[3*j] += grad_j.x;
                    gradient[3*j + 1] += grad_j.y;
                    gradient[3*j + 2] += grad_j.z;
                }
                
                // Hessian contribution (simplified - full version needs second derivatives)
                float scale = hess_g / (rest_length * rest_length);
                
                for (int dim = 0; dim < 3; dim++) {
                    float h_val = scale * direction[dim] * direction[dim];
                    
                    if (!fixeds_.count(i)) {
                    triplets.push_back(Eigen::Triplet<float>(3*i + dim, 3*i + dim, h_val));
                    }
                    if (!fixeds_.count(j)) {
                    triplets.push_back(Eigen::Triplet<float>(3*j + dim, 3*j + dim, h_val));
                    }
                    
                    // Off-diagonal terms
                    if (!fixeds_.count(i) && !fixeds_.count(j)) {
                    triplets.push_back(Eigen::Triplet<float>(3*i + dim, 3*j + dim, -h_val));
                    triplets.push_back(Eigen::Triplet<float>(3*j + dim, 3*i + dim, -h_val));
                    }
                }
                }
            }




        private:
            int n_;
            // https://www.reddit.com/r/cpp_questions/comments/u4n4kn/why_i_can_hash_stdpair_to_use_as_a_key_in_stdset/
            std::map<std::pair<int, int>, std::pair<float, float>> ks_rs_;
            std::map<int, float> masses_;
            std::map<int, float> dampings_;
            std::set<int> fixeds_;
            bool wind_ = false;

        
        
        
        
    };

    
}  // namespace GLOO

#endif
