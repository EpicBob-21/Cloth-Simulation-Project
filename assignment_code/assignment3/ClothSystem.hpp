// // ClothSystem.hpp
// #ifndef CLOTH_SYSTEM_H_
// #define CLOTH_SYSTEM_H_

// #include "ParticleState.hpp"
// #include "ParticleSystemBase.hpp"
// #include <vector>
// #include <glm/glm.hpp>

// namespace GLOO {

// struct MeshData {
//     std::vector<glm::vec3> rest_positions;
//     std::vector<glm::ivec3> triangles;
//     std::vector<glm::ivec2> edges;
//     std::vector<float> vertex_masses;
//     std::vector<int> fixed_vertices;
// };

// class ClothSystem : public ParticleSystemBase {
// private:
//     MeshData mesh_;
//     float youngs_modulus_;
//     float poisson_ratio_;
//     float g_hat_;  // Contact gap
//     float strain_limit_;
    
// public:
//     ClothSystem() : youngs_modulus_(1e6), poisson_ratio_(0.25), 
//                     g_hat_(0.001f), strain_limit_(0.05f) {}
    
//     // For Forward Euler (keep for compatibility, but won't use)
//     ParticleState ComputeTimeDerivative(const ParticleState& state,
//                                         float time) const override {
//         // Not used for Newton method, but keep for interface
//         ParticleState derivative;
//         // ... can leave empty or implement simple version
//         return derivative;
//     }
    
//     // NEW: Methods for Newton solver
//     const MeshData& GetMesh() const { return mesh_; }
//     MeshData& GetMesh() { return mesh_; }
    
//     float GetYoungsModulus() const { return youngs_modulus_; }
//     float GetPoissonRatio() const { return poisson_ratio_; }
//     float GetContactGap() const { return g_hat_; }
//     float GetStrainLimit() const { return strain_limit_; }
    
//     void SetFixedVertex(int vertex_idx) {
//         mesh_.fixed_vertices.push_back(vertex_idx);
//     }
    
//     void AddTriangle(int v0, int v1, int v2) {
//         mesh_.triangles.push_back(glm::ivec3(v0, v1, v2));
//     }
    
//     void AddEdge(int v0, int v1) {
//         mesh_.edges.push_back(glm::ivec2(v0, v1));
//     }
    
//     void SetVertexMass(int idx, float mass) {
//         if (idx >= mesh_.vertex_masses.size()) {
//             mesh_.vertex_masses.resize(idx + 1);
//         }
//         mesh_.vertex_masses[idx] = mass;
//     }
    
//     void InitializeRestPositions(const std::vector<glm::vec3>& positions) {
//         mesh_.rest_positions = positions;
//     }
// };

// }  // namespace GLOO

// #endif