#ifndef BALL_SYSTEM_H_
#define BALL_SYSTEM_H_

#include "ParticleState.hpp"
#include "ParticleSystemBase.hpp"

namespace GLOO {
class BallSystem : public ParticleSystemBase {
 public:

    ParticleState BallSystem::ComputeTimeDerivative(const ParticleState& state, float time) const override{
        return {{glm::vec3(-state.positions[0].y, state.positions[0].x, 0.0f)}, {glm::vec3(0.0f)}};
    }
};
}  // namespace GLOO

#endif
