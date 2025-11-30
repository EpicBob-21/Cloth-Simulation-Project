#ifndef PENDULUM_SYSTEM_H_
#define PENDULUM_SYSTEM_H_

#include "ParticleState.hpp"
#include "ParticleSystemBase.hpp"

namespace GLOO {
class PendulumSystem : public ParticleSystemBase {
 public:
    //change to handle multiple particles!!
    ParticleState PendulumSystem::GravityForce(const ParticleState& state)const{
        size_t n = state.positions.size();
        ParticleState gravity_state;
        for(size_t i=0;i<n;i++){
            gravity_state.positions.push_back(glm::vec3(0.0f));
            gravity_state.velocities.push_back(glm::vec3(0.0f, -9.8f, 0.0f));
        }
        return gravity_state;
    }

    ParticleState PendulumSystem::DragForce(const ParticleState& state, float k, float m)const{
        size_t n = state.positions.size();
        ParticleState drag_state;
        for(size_t i=0;i<n;i++){
            drag_state.positions.push_back(glm::vec3(0.0f));
            drag_state.velocities.push_back(-k*state.velocities[i]/m);
        }
        return drag_state;
    }

    ParticleState PendulumSystem::SpringForce(const ParticleState& state, float m)const{
        ParticleState spring_state;
        size_t n = state.positions.size();

        for(size_t i=0;i<n;i++){
            spring_state.positions.push_back(glm::vec3(0.0f));
            spring_state.velocities.push_back(glm::vec3(0.0f));
        }

        for (glm::vec4 this_spring:state.springs) {
            int i = static_cast<int>(this_spring[0]);
            int j = static_cast<int>(this_spring[1]);
            float k = this_spring[2];
            float r = this_spring[3];
            glm::vec3 dvec = state.positions[i] - state.positions[j];
            float dist = glm::length(dvec);
            if (dist > 0.00001f) {
                glm::vec3 s = -k * (dist - r) * (dvec / dist);
                spring_state.velocities[i] += s/m;
                spring_state.velocities[j] -= s/m;
            }
        }
        return spring_state;

    }

    ParticleState PendulumSystem::Velocities(const ParticleState& state) const{
        ParticleState position_state;
        size_t n = state.positions.size();
        for(size_t i=0;i<n;i++){
            position_state.positions.push_back(state.velocities[i]);
            position_state.velocities.push_back(glm::vec3(0.0f));
        }
        return position_state;
    }

    ParticleState PendulumSystem::ComputeTimeDerivative(const ParticleState& state, float time) const override{
        ParticleState new_state = GravityForce(state) + DragForce(state, state.drag, 0.005f) + SpringForce(state, 0.005f) + Velocities(state);
        new_state.springs = state.springs;
        new_state.fixed_pts = state.fixed_pts;
        new_state.drag = state.drag;
        for(int pt:state.fixed_pts){
            new_state.positions[pt] = glm::vec3(0.0f);
            new_state.velocities[pt] = glm::vec3(0.0f);
        }
        return new_state;
    }
};
}  // namespace GLOO

#endif
