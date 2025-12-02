#ifndef INTEGRATOR_FACTORY_H_
#define INTEGRATOR_FACTORY_H_

#include "IntegratorBase.hpp"
#include "ForwardRK4Integrator.hpp"
#include "AdaptiveRK45Integrator.hpp"
#include "ForwardEulerIntegrator.hpp"
#include "ForwardTrapezoidIntegrator.hpp"
#include "CubicBarrierIntegrator.hpp"

#include <stdexcept>

#include "gloo/utils.hpp"

#include "IntegratorType.hpp"

namespace GLOO {
class IntegratorFactory {
 public:
  template <class TSystem, class TState>
  static std::unique_ptr<IntegratorBase<TSystem, TState>> CreateIntegrator(
      IntegratorType type) {
        if (type == IntegratorType::Euler) {
          return make_unique<ForwardEulerIntegrator<TSystem, TState>>();
        } else if (type == IntegratorType::Trapezoidal) {
          return make_unique<ForwardTrapezoidIntegrator<TSystem, TState>>();
        } else if (type == IntegratorType::RK4) {
          return make_unique<ForwardRK4Integrator<TSystem, TState>>();
        } else if (type == IntegratorType::AdaptiveRK45) {
          return make_unique<AdaptiveRK45Integrator<TSystem, TState>>();
        } else if (type == IntegratorType::CubicBarrier) {
          return make_unique<CubicBarrierIntegrator<TSystem, TState>>();
        } else {
          throw std::runtime_error("Unknown integrator type!");
        }
  }
};
}  // namespace GLOO

#endif
