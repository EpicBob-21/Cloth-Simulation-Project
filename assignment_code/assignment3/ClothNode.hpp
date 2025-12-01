#ifndef CLOTH_NODE_H_
#define CLOTH_NODE_H_

#include "ParticleState.hpp"
#include "ParticleSystemBase.hpp"
#include "IntegratorBase.hpp"
#include "IntegratorType.hpp"
#include "PendulumSystem.hpp"

#include "gloo/SceneNode.hpp"
#include "gloo/VertexObject.hpp"
#include "gloo/shaders/ShaderProgram.hpp"
#include "gloo/components/MaterialComponent.hpp"



namespace GLOO {
    class ClothNode : public SceneNode {
        public:
            ClothNode(IntegratorType integrator_type, glm::vec3 material, float h);
            void Update(double dt) override;
            void Restart();
            
        private:
            float h_;
            std::shared_ptr<VertexObject> point_mesh_;
            std::shared_ptr<ShaderProgram> shader_;
            std::shared_ptr<Material> material_;
            std::shared_ptr<Material> line_material_;
            std::unique_ptr<IntegratorBase<PendulumSystem, ParticleState>> integrator_;
            std::unique_ptr<ParticleState> state_;
            std::unique_ptr<PendulumSystem> system_;
            std::vector<SceneNode*> points_;
            std::shared_ptr<VertexObject> all_line_;
        
    };
}

#endif