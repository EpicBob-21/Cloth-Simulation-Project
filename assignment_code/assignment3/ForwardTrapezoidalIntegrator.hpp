#ifndef FORWARD_TRAPEZOIDAL_INTEGRATOR_H_
#define FORWARD_TRAPEZOIDAL_INTEGRATOR_H_

#include "IntegratorBase.hpp"

namespace GLOO {
template <class TSystem, class TState>
class ForwardTrapezoidalIntegrator : public IntegratorBase<TSystem, TState> {
  TState Integrate(const TSystem& system,
                   const TState& state,
                   float start_time,
                   float dt) const override {
    // TODO: Here we are returning the state at time t (which is NOT what we
    // want). Please replace the line below by the state at time t + dt using
    // forward Trapezoidal integration.

    auto f1 = system.ComputeTimeDerivative(state, start_time);
    auto f2 = system.ComputeTimeDerivative(state+(dt*f1), start_time+dt);

    return state + dt/2*(f1+f2);

  }
};
}  // namespace GLOO

#endif
