#ifndef FORWARD_TRAPEZOID_INTEGRATOR_H_
#define FORWARD_TRAPEZOID_INTEGRATOR_H_

#include "IntegratorBase.hpp"

namespace GLOO {
template <class TSystem, class TState>
class ForwardTrapezoidIntegrator : public IntegratorBase<TSystem, TState> {
  TState Integrate(const TSystem& system,
                   const TState& state,
                   float start_time,
                   float dt) const override {
    //
    auto f_0 = system.ComputeTimeDerivative(state, start_time);
    auto f_1 = system.ComputeTimeDerivative(state + dt * f_0, start_time + dt);
    TState new_state = state + dt/2 * (f_0 + f_1);
    return new_state;
  }
};
}  // namespace GLOO

#endif
