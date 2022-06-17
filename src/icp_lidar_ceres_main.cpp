#include "icp_lidar_ceres/icp_lidar_ceres.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

std::vector<double> eigenVec2stdVec(VectorXd evec) {
    std::vector<double> svec;
    for(int i = 0; i < evec.size(); i++) {
        svec.push_back(evec[i]);
    }
    return svec;
}

int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);

    icp_lidar_ceres icp(argv[1], argv[2]);
    MatrixXd result_points = icp.icp_non_linear(icp.reference_points, icp.points_to_be_aligned, 20, 10, 8, false);
    // cout << "icp.reference_points:" << endl << icp.reference_points << endl << endl;
    // cout << "icp.points_to_be_aligned:" << endl << icp.points_to_be_aligned << endl << endl;
    // cout << "result_points:" << endl << result_points << endl << endl;

    std::vector<double> ref_x = eigenVec2stdVec(icp.reference_points.block<1,30>(0,0));
    std::vector<double> ref_y = eigenVec2stdVec(icp.reference_points.block<1,30>(1,0));

    std::vector<double> input_x = eigenVec2stdVec(icp.points_to_be_aligned.block<1,30>(0,0));
    std::vector<double> input_y = eigenVec2stdVec(icp.points_to_be_aligned.block<1,30>(1,0));

    std::vector<double> result_x = eigenVec2stdVec(result_points.block<1,30>(0,0));
    std::vector<double> result_y = eigenVec2stdVec(result_points.block<1,30>(1,0));
    plt::scatter(ref_x, ref_y, 'r');
    plt::plot(input_x, input_y, "b*");
    plt::plot(result_x, result_y, "r*");
    plt::show();
    return 0;
}