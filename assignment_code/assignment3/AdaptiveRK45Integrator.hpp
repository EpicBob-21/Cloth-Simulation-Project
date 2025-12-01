#ifndef ADAPTIVE_RK45_INTEGRATOR_H_
#define ADAPTIVE_RK45_INTEGRATOR_H_

#include "IntegratorBase.hpp"

namespace GLOO {
template <class TSystem, class TState>
class AdaptiveRK45Integrator : public IntegratorBase<TSystem, TState> {
  TState Integrate(const TSystem& system,
                   const TState& state,
                   float start_time,
                   float dt) const override {
    // https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta%E2%80%93Fehlberg_method, formula 2
    // https://math.stackexchange.com/questions/3303823/adaptive-runge-kutta-fehlberg-method-constant

    
    auto k_1 = system.ComputeTimeDerivative(state, start_time);
    auto k_2 = system.ComputeTimeDerivative(state + dt/4.0f * k_1, start_time + dt/4.0f);
    auto k_3 = system.ComputeTimeDerivative(state + dt * ((3.0f/32.0f) * k_1 + (9.0f/32.0f) * k_2), start_time + 3.0f/8.0f*dt);
    auto k_4 = system.ComputeTimeDerivative(
        state + dt * (
        + (1932.0f/2197.0f) * k_1
        + (-7200.0f/2197.0f) * k_2 
        + (7296.0f/2197.0f) * k_3), 
        start_time + 12.0f/13.0f*dt);
    		
    auto k_5 = system.ComputeTimeDerivative(
        state + dt * (
        + (439.0f/216.0f) * k_1 
        + -8 * k_2 
        + (3680.0f/513.0f) * k_3 
        + (-845.0f/4104.0f) * k_4), 
        start_time + dt);

    auto k_6 = system.ComputeTimeDerivative(
        state + dt * (
        + (-8.0f/27.0f) * k_1 
        + 2 * k_2 
        + (-3544.0f/2565.0f) * k_3 
        + (1859.0f/4104.0f) * k_4
        + (-11.0f/40.0f) * k_5), 
        start_time + dt/2.0f);

    TState fourth = state + dt * ((25.0f/216)*k_1 + (1408.0f/2565)*k_3 + (2197.0f/4104)*k_4 + (-1.0f/5)*k_5);
    TState fifth = state + dt * ((16.0f/135)*k_1 + (6656.0f/12825)*k_3 + (28561.0f/56430)*k_4 + (-9.0f/50)*k_5 + (2.0f/55)*k_6);
    TState error = (fifth + (-1) * fourth);


    // how to calculate error of all?? find largest one??
    float epsilon = 1e-6;
    for (int i = 0; i < error.positions.size(); i++) {
      float e = glm::length(error.positions[i]);
      if (e > epsilon) {
        return Integrate(system, state, start_time, 0.9f * dt * pow((epsilon / e), 0.2f));
      }
    }

    return fifth;
  }
};
}  // namespace GLOO

#endif
