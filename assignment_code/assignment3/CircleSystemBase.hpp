#ifndef CIRCLE_SYSTEM_BASE_H_
#define CIRCLE_SYSTEM_BASE_H_

#include "ParticleState.hpp"

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

    };
}  // namespace GLOO

#endif
