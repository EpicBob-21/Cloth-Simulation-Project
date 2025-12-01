#include "PendulumSystem.hpp"
#include "ParticleState.hpp"
#include "ParticleSystemBase.hpp"



// More include here
namespace GLOO {
    ParticleState PendulumSystem::ComputeTimeDerivative(const ParticleState& state, float time) const {
        auto n = state.positions.size();
        glm::vec3 F;
        ParticleState derivative;
        for (int i = 0; i < n; i++) {
            derivative.positions.push_back(state.velocities[i]);
            // here I used claude because I didn't know why masses_[i] was giving error
            glm::vec3 W = glm::vec3(0.0f);

            if (wind_) {
                W = glm::vec3(0.0f, 10.0f, 0.0f);
            }

            glm::vec3 G = glm::vec3(0.0f, -9.81f * masses_.at(i), 0.0f);

            glm::vec3 D = state.velocities[i] * -dampings_.at(i);

            derivative.velocities.push_back((W + G + D) / masses_.at(i));
        }

        for (const auto& [key, value] : ks_rs_) {
            int i = key.first;
            int j = key.second;
            float k = value.first;
            float r = value.second;

            glm::vec3 d = state.positions[i] - state.positions[j];
            float length = glm::length(d);

            glm::vec3 spring_force = -k * (length - r) * d/length;

            derivative.velocities[i] += spring_force / masses_.at(i);
            derivative.velocities[j] -= spring_force / masses_.at(j);
        }

        for (const auto& fixed_ : fixeds_) {
            derivative.velocities[fixed_] = glm::vec3(0.0f);
            derivative.positions[fixed_] = glm::vec3(0.0f);
        }
        
        return derivative;

    }

    
}
