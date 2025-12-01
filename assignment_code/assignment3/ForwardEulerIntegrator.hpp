#ifndef FORWARD_EULER_INTEGRATOR_H_
#define FORWARD_EULER_INTEGRATOR_H_

#include "IntegratorBase.hpp"

namespace GLOO {
template <class TSystem, class TState>
class ForwardEulerIntegrator : public IntegratorBase<TSystem, TState> {
  TState Integrate(const TSystem& system,
                   const TState& state,
                   float start_time,
                   float dt) const override {
    // TODO: Here we are returning the state at time t (which is NOT what we
    // want). Please replace the line below by the state at time t + dt using
    // forward Euler integration.
    // std::cout << "Old Position: " << state.positions[0][0] << ", " << state.positions[0][1] << ", " << state.positions[0][2] << std::endl;
    auto f = system.ComputeTimeDerivative(state, start_time);
    TState new_state = state + dt * f;
    return new_state;
  }
};
}  // namespace GLOO

#endif
