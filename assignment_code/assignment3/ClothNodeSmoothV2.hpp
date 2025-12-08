#ifndef CLOTH_NODE_V2_SMOOTH_H_
#define CLOTH_NODE_V2_SMOOTH_H_

#include "ParticleState.hpp"
#include "ParticleSystemBase.hpp"
#include "IntegratorBase.hpp"
#include "IntegratorType.hpp"
#include "PendulumSystem.hpp"
#include "ClothSystemV2.hpp"

#include "gloo/SceneNode.hpp"
#include "gloo/VertexObject.hpp"
#include "gloo/shaders/ShaderProgram.hpp"
#include "gloo/components/MaterialComponent.hpp"



namespace GLOO {
    class ClothNodeSmoothV2 : public SceneNode {
        public:
            ClothNodeSmoothV2(IntegratorType integrator_type, glm::vec3 material, glm::vec3 top_left_pos, float h, int dim = 8);
            void Update(double dt) override;
            void Restart();

        private:
            float h_;
            int DIM;
            glm::vec3 top_left_corner_pos_;
            std::shared_ptr<VertexObject> point_mesh_;
            std::shared_ptr<VertexObject> cloth_mesh_;
            std::shared_ptr<ShaderProgram> shader_;
            std::shared_ptr<Material> material_;
            std::shared_ptr<Material> line_material_;
            std::unique_ptr<IntegratorBase<ClothSystemV2, ParticleState>> integrator_;
            std::unique_ptr<ParticleState> state_;
            std::unique_ptr<ClothSystemV2> system_;
            std::vector<SceneNode*> points_;
            std::shared_ptr<VertexObject> all_line_;

    };
}

#endif
