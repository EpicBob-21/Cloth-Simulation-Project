#ifndef PENDULUM_SYSTEM_H_
#define PENDULUM_SYSTEM_H_

#include "ParticleState.hpp"
#include "ParticleSystemBase.hpp"
#include <map>


namespace GLOO {
    class PendulumSystem : public ParticleSystemBase {
        public:
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
                fixeds_.push_back(i);
            }

            void Blow() {
                wind_ = !wind_;
            }

        private:
            int n_;
            // https://www.reddit.com/r/cpp_questions/comments/u4n4kn/why_i_can_hash_stdpair_to_use_as_a_key_in_stdset/
            std::map<std::pair<int, int>, std::pair<float, float>> ks_rs_;
            std::map<int, float> masses_;
            std::map<int, float> dampings_;
            std::vector<int> fixeds_;
            bool wind_ = false;
    };
}  // namespace GLOO

#endif
