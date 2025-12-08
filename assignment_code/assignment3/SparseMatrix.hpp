// #ifndef SPARSE_MATRIX_H_
// #define SPARSE_MATRIX_H_

// #include <vector>
// #include <stdexcept>
// #include "ParticleState.hpp"


// #include <glm/glm.hpp>

// namespace GLOO {
// class SparseMatrix {
// private:
//     std::vector<Eigen::Triplet<float>> triplets_;
//     Eigen::SparseMatrix<float> matrix_;
//     int size_;
    
// public:
//     SparseMatrix(int size) : size_(size) {}
    
//     void AddDiagonalBlock(int i, int j, const glm::mat3& block);
//     void AddBlock(int i, int j, const glm::mat3& block);
//     void MakePositiveDefinite();  // Ensure SPD
//     glm::vec3 Multiply(const glm::vec3& v) const;
// };

// // PCG solver with block-Jacobi preconditioner
// class PCGSolver {
// private:
//     float tolerance_ = 1e-3f;
//     int max_iterations_ = 1000;
    
// public:
//     std::vector<glm::vec3> Solve(const SparseMatrix& A,
//                                  const std::vector<glm::vec3>& b);
// };

// // Line search with CCD (Continuous Collision Detection)
// class LineSearch {
// public:
//     float Search(const ParticleState& state,
//                 const std::vector<glm::vec3>& direction);
    
// private:
//     bool CheckCollisions(const ParticleState& state,
//                         const std::vector<glm::vec3>& direction,
//                         float alpha);
// };

// // Collision detection
// class CollisionDetector {
// public:
//     std::vector<ContactPair> DetectContacts(const ParticleState& state,
//                                            float g_hat);
    
// private:
//     // Spatial hashing or BVH for broad phase
//     // Exact distance computation for narrow phase
// };

// }  // namespace GLOO


// #endif
