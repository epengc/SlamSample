#include <iostream>
#include <stdint.h>
#include <stdlib.h>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <unordered_set>
#include <memory>
#include <vector>

#include "g2o/stuff/sampler.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/batch_stats.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_dogleg.h"

#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/types/sba/types_six_dof_expmap.h"

#include "g2o/solvers/structure_only/structure_only_solver.h"

#include "common/BundleParams.h" 
#include "common/BALProblem.h"
#include "g2o_bal_class.h"

using namespace Eigen;
using namespace std;

typedef Eigen::Map<Eigen::VectorXd> VectorRef;
typedef Eigen::Map<const Eigen::VectorXd> ConstVectorRef;
typedef g2o::BlockSolver<g2o::BlockSolverTraits<9,3>> BalBlockSolver;

// Setup the vectexs and edges for the bundle adjustment.
void SolveProblem(const char* filename, const BundleParams& params);
void BuildProblem(const BALProblem* bal_problem, g2o::SparseOptimizer* optimizer, const BundleParams& params);

int main(int argc, char** argv)
{
    // Set parameters here
    BundleParams params(argc, argv);

    if(params.input.empty()){
        std::cout<<"Usage: bundle_adjuster - input <path for dataset>";
        return 1;
    }
    SolveProblem(params.input.c_str(), params);
    return 0;
    
}

void SolveProblem(const char* filename, const BundleParams& params)
{
    BALProblem bal_problem(filename);
    // Show some information here...
    std::cout<<"bal problem is loaded ..."<<std::endl;
    std::cout<<"bal problem have "<<bal_problem.num_cameras()<<" cameras and "<<bal_problem.num_points()<<" points"<<endl;
    std::cout<<"Forming "<<bal_problem.num_observations()<<" observations. "<<std::endl;
    
    // Store the initial 3D cloud and camera pose..
    if(!params.initial_ply.empty()) bal_problem.WriteToPLFile(params.initial_ply);
    std::cout<<"beginning problem ..."<<std::endl;

    // Add some noise for the inital value
    srand(params.random_seed);
    bal_problem.Normalize();
    bal_problem.Normalize();
    bal_problem.Perturb(params.rotation_sigma, params.translation_sigma, params.point_sigma);

    std::cout<<"Normalization complete"<<std::endl;
    
    g2o::SparseOptimizer optimizer;
    SetSolverOptionsFromFlags(&bal_problem, params, &optimizer);
    BuildProblem(&bal_problem, &optmizer, params);

    std::cout<<"Begin optimization"<<std::endl;
    // Perform the optimization
    optimizer.initializeOptimization();
    optimizer.setVerbose(true);
    optimizer.optimize(params.num_iterations);

    std::cout<<"Optimization Complete..."<<std::endl;
    // Write the optimized data into BALPorblem class
    WriteToBALProblem(&bal_problem, &optimizer);

    // Write the result into a .ply file.
    if(!params.final_ply.empty()) bal_problem.WriteToPLFile(params.final_ply);
}

void BuildProblem(const BALProblem* bal_problem, g2o::SparseOptimizer* optimizer, const BundleParams& params)
{
    const int num_points = bal_problem->num_points();
    const int num_cameras = bal_problem->num_cameras();
    const int camera_block_size = bal_problem->camera_block_size();
    const int point_block_size = bal_problem->point_block_size();

    // Set camera vertex with initial value in the dataset
    const double* raw_cameras = bal_problem->cameras();
    for(size_t i=0; i<num_cameras; ++i){
        ConstVectorRef temVecCamera(raw_cameras + camera_block_size*i, camera_block_size);
        VertexCameraBAL* pCamera = new VertexCameraBAL();
        // Initial value for the camera i...
        pCamera->setEstimate(temVecCamera);
        // Set id for each camera vertex
        pCamera->setId(i);
        // Remeber to add vertex into optimizer
        optimizer->addVertex(pCamera);
    }

    // Set points vertex with initial value in the dataset
    const double* raw_points = bal_problem->points();
    for(size_t i=0; i<num_points; ++i){
        ConstVectorRef temVecPoint(raw_points+point_block_size*i, point_block_size);
        VertexPointBAL* pPoint = new VertexPointBAL();
        // Initial value for the point i ...
        pPoint->setEstimate(temVecPoint);
        // Each Vertex shoul dhave an unique id, no matter it is a camera vertex, or a point vertex
        pPoint->setId(j+num_cameras);
        // Remember to add Vertex into optimizer ...
        pPoint->setMarginalized(true);
        optimizer->addVertex(pPoint);
    }

}
