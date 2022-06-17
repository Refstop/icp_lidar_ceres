#ifndef ICP_LIDAR_CERES
#define ICP_LIDAR_CERES

#include <iostream>
#include <knncpp.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <glog/logging.h>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <cmath>
#define RAD2DEG(rad) rad*(180/M_PI)
#define DEG2RAD(deg) deg*(M_PI/180)

using namespace std;
using namespace Eigen;
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

typedef knncpp::Matrixi Matrixi;

class icp_lidar_ceres {
    public:
    MatrixXd reference_points, points_to_be_aligned;
    icp_lidar_ceres(string ref_file = "\0", string tobealigned_file = "\0");
    ~icp_lidar_ceres() {}
    void knn_kdtree(const MatrixXd& reference_points, const MatrixXd& points_to_be_aligned);
    // point to plane icp
    MatrixXd icp_non_linear(const MatrixXd& reference_points, const MatrixXd& points, int max_iterations = 10, double distance_threshold = 0.3, int point_pairs_threshold=8, bool verbose=true);

    private:
    Matrixi indices_;
    MatrixXd distances_;
    std::vector<Vector2d> compute_normals_(const MatrixXd& reference_points);
    Vector2d split_(string input, char delimiter);
    void push_back_(MatrixXd& m, Vector2d&& values, std::size_t row);
    double** pointsMatrix_to_2darray_(const MatrixXd& points);
};

#endif