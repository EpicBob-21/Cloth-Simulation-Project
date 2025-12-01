#ifndef CIRCLE_SPHERE_NODE_H_
#define CIRCLE_SPHERE_NODE_H_

#include "ParticleState.hpp"
#include "ParticleSystemBase.hpp"
#include "IntegratorBase.hpp"
#include "IntegratorType.hpp"
#include "CircleSystemBase.hpp"

#include "gloo/SceneNode.hpp"
#include "gloo/VertexObject.hpp"
#include "gloo/shaders/ShaderProgram.hpp"
#include "gloo/components/MaterialComponent.hpp"



namespace GLOO {
    class CircleSphereNode : public SceneNode {
        public:
            CircleSphereNode(IntegratorType integrator_type, glm::vec3 material, float h);
            void Update(double dt) override;
            
        private:
            float h_;
            std::shared_ptr<VertexObject> sphere_mesh_;
            std::shared_ptr<ShaderProgram> shader_;
            std::shared_ptr<Material> material_;
            std::unique_ptr<IntegratorBase<CircleSystemBase, ParticleState>> integrator_;
            std::unique_ptr<ParticleState> state_;
            std::unique_ptr<CircleSystemBase> system_;
        
    };
}

#endif