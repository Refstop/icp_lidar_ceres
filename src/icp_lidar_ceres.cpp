#include "icp_lidar_ceres/icp_lidar_ceres.h"

class CostFunctor {
    public:
    /*
        p_point: double[3], reference point
        q_point: double[3], point to be aligned
        n: double[3], normal vector of q_point
    */
    CostFunctor(const double* p_point, const double* q_point, const double* normal)
    :p_point_(p_point[0], p_point[1]), q_point_(q_point[0], q_point[1]), normal_(normal[0], normal[1]) {}
    /*
        Template param T: double[]
        x: double[3], information of R,t formed by (x,y,theta)
        residual: double, error value
        여기서의 T는 Jet 함수와 관련이 있는 것인가??
    */
    template<typename T>
    bool operator()(const T *const x, T *residual) const {
        // Matrix2d R에 집어넣으려고 하면 그냥 double이 아닌 Jet 형식이라서 안된다고 한다.
        // ceres::Jet<double, 6>이라는 타입이 되기 때문인 것 같음
        Eigen::Matrix<T, 2, 2> R;
        int weight;
        const T cos = ceres::cos(x[2]);
        const T sin = ceres::sin(x[2]);
        R << cos, -sin,
            sin, cos;
        Eigen::Matrix<T, 2, 1> t(x[0], x[1]);
        Eigen::Matrix<T, 2, 1> e_temp = R * p_point_ + t - q_point_;
        if(e_temp.norm () > 0.001) weight = 0;
        else weight = 1;
        residual[0] = normal_.transpose() * weight * e_temp;
        // residual[0] = normal_.transpose() * (R * p_point_ + t - q_point_);
        return true;
    }
    private:
    Vector2d p_point_, q_point_, normal_;
};

class CostFunctor_p2p {
    public:
    CostFunctor_p2p(const double* p_point, const double* q_point)
    :p_point_(p_point[0], p_point[1]), q_point_(q_point[0], q_point[1]) {}

    template<typename T>
    bool operator()(const T *const x, T *residual) const {
        Eigen::Matrix<T, 2, 2> R; 
        const T cos = ceres::cos(x[2]);
        const T sin = ceres::sin(x[2]);
        R << cos, -sin,
            sin, cos;
        Eigen::Matrix<T, 2, 1> t(x[0], x[1]);
        auto res = R * p_point_ + t - q_point_;
        residual[0] = res[0];
        residual[1] = res[1];
        return true;
    }
    private:
    Vector2d p_point_, q_point_;
};

class QuadraticCostFunction : public ceres::SizedCostFunction<1, 3> {
    public:
    QuadraticCostFunction(const double* p_point, const double* q_point, const double* normal, int i)
    :p_point_(p_point[0], p_point[1]), q_point_(q_point[0], q_point[1]), normal_(normal[0], normal[1]), index_(i) {}
    virtual ~QuadraticCostFunction() {}
    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
        cout << "index_: " << index_ << endl;
        // cout << "here1" << endl;
        const double x = parameters[0][0];
        const double y = parameters[0][1];
        const double th = parameters[0][2];
        
        Eigen::Matrix2d R; 
        R << ceres::cos(th), -ceres::sin(th),
            ceres::sin(th), ceres::cos(th);
        Eigen::Vector2d t(x, y);
        residuals[0] = normal_.transpose() * (R * p_point_ + t - q_point_);
        // cout << "here2" << endl;
        if (jacobians != nullptr && jacobians[0] != nullptr) {
            jacobians[0][0] = 1;
            jacobians[0][1] = 0;
            jacobians[0][2] = -ceres::sin(th) * p_point_(0) - ceres::cos(th) * p_point_(1);

            jacobians[0][3] = 0;
            jacobians[0][4] = 1;
            jacobians[0][5] = ceres::cos(th) * p_point_(0) - ceres::sin(th) * p_point_(1);

            jacobians[0][6] = 0;
            jacobians[0][7] = 0;
            jacobians[0][8] = 1;
        }
        // cout << "here3" << endl;
        return true;
    }
    private:
    Vector2d p_point_, q_point_, normal_;
    int index_;
};

icp_lidar_ceres::icp_lidar_ceres(string ref_file, string tobealigned_file) {
    if(ref_file != "\0" && tobealigned_file != "\0") {
        this->reference_points.resize(1,2);
        this->points_to_be_aligned.resize(1,2);

        ifstream reference_points(ref_file);
	    if(!reference_points.is_open()) {
            cout << "Error opening file" << endl;
            exit(1);
        }
        int i = 0;
	    for(string line; reference_points.peek() != EOF; i++) {
            getline(reference_points, line);
            push_back_(this->reference_points, split_(line, ' '), i);
        }

        ifstream points_to_be_aligned(tobealigned_file);
	    if(!points_to_be_aligned.is_open()) {
            cout << "Error opening file" << endl;
            exit(1);
        }

        i = 0;
	    for(string line; points_to_be_aligned.peek() != EOF; i++) {
            getline(points_to_be_aligned, line);
            push_back_(this->points_to_be_aligned, split_(line, ' '), i);
        }
        this->reference_points.transposeInPlace();
        this->points_to_be_aligned.transposeInPlace();
    }
}

void icp_lidar_ceres::knn_kdtree(const MatrixXd& reference_points, const MatrixXd& points_to_be_aligned) {
    knncpp::KDTreeMinkowskiX<double, knncpp::EuclideanDistance<double>> kdtree(reference_points);

    kdtree.setBucketSize(1);
    kdtree.build();

    kdtree.query(points_to_be_aligned, 1, indices_, distances_);
}

MatrixXd icp_lidar_ceres::icp_non_linear(const MatrixXd& reference_points, const MatrixXd& points, int max_iterations, double distance_threshold,
        int point_pairs_threshold, bool verbose) {
    std::vector<Vector2d> normals = compute_normals_(reference_points);
    double x[3] = {0,};
    MatrixXd x_tf = MatrixXd::Identity(3,3);
    MatrixXd result_points = points;

    for(int iter_num = 0; iter_num < max_iterations; iter_num++) {
        knn_kdtree(reference_points, result_points);

        MatrixXd points_pair_a(1,2), points_pair_b(1,2);
        int nn_index = 0;
        for(int i = 0; i < distances_.size(); i++) {
            if(distances_(nn_index) < distance_threshold) {
                push_back_(points_pair_a, result_points.block<2,1>(0,nn_index), nn_index);
                push_back_(points_pair_b, reference_points.block<2,1>(0,indices_(nn_index)), nn_index);
                nn_index++;
            }
        }

        double** p_points = pointsMatrix_to_2darray_(points_pair_a);
        double** q_points = pointsMatrix_to_2darray_(points_pair_b);

        Problem problem;
        /*
            Template of AutoDiffCostFunction
            First number of <CostFunctor, 1, 3> means size of residual
            numbers after first one means sizes of variable(double array) that compose cost function of optimization
            i.e. <CostFunctor, 1, 3> <=> {size of residual, size of x}
        */
        for(int i = 0; i < points_pair_a.rows(); i++) {
            double normal[2] = {normals[i][0], normals[i][1]};
            cout << "i: " << i << endl;
            // CostFunction *cost_function
            //     = new AutoDiffCostFunction<CostFunctor, 1, 3>(new CostFunctor(p_points[i], q_points[i], normal));
                // = new AutoDiffCostFunction<CostFunctor_p2p, 2, 3>(new CostFunctor_p2p(p_points[i], q_points[i]));
            CostFunction *cost_function
                = new QuadraticCostFunction(p_points[i], q_points[i], normal, i);
            problem.AddResidualBlock(cost_function, NULL, x);
        }
        
        
        Solver::Options options;
        // options.minimizer_progress_to_stdout = true;
        Solver::Summary summary;
        ceres::Solve(options, &problem, &summary); // here
        if(verbose) {
            std::cout << "Corres Iter: " << iter_num << endl;
            std::cout << summary.BriefReport();
            std::cout << " x: ";
            for(int i=0; i<3; i++) std::cout << x[i] << ' ';
            std::cout << "\n";
        }
        Matrix2d R;
        R << ceres::cos(x[2]), -ceres::sin(x[2]),
            ceres::sin(x[2]), ceres::cos(x[2]);
        MatrixXd t(2, result_points.cols());
        t << MatrixXd::Constant(1,result_points.cols(),x[0]), MatrixXd::Constant(1,result_points.cols(),x[1]);
        Matrix3d cur_tf;
        cur_tf.block<2,2>(0,0) = R;
        cur_tf.block<2,1>(0,2) = Vector2d(x[0], x[1]);
        cur_tf.block<1,3>(2,0) = Vector3d(0,0,1);
        if(verbose) cout << cur_tf << endl;
        x_tf *= cur_tf;
        MatrixXd temp1(3, result_points.cols()), temp2;
        temp1 << result_points, MatrixXd::Constant(1,result_points.cols(),1);
        temp2 = cur_tf*temp1;
        result_points = temp2.topRows<2>();

        for(int i = 0; i < points_pair_a.rows(); i++) {
            delete [] *(p_points + i);
            delete [] *(q_points + i);
        }
        delete [] p_points;
        delete [] q_points;
    }
    
    if(verbose) {
        cout << x_tf*Vector3d(points(0,0), points(1,0), 1) << endl;
        cout << reference_points.block<2,1>(0,0) << endl;
    }
    return result_points;
}

/*
    reference_point: (x,y)
*/
std::vector<Vector2d> icp_lidar_ceres::compute_normals_(const MatrixXd& reference_points) {
    std::vector<Vector2d> normals;
    normals.push_back(Vector2d(0,0));
    Vector2d normal;
    for(int i = 1; i < reference_points.cols()-1; i++) {
        double dx = reference_points(0,i+1) - reference_points(0,i-1);
        double dy = reference_points(1,i+1) - reference_points(1,i-1);
        
        normal << -dy, dx;
        normal = normal/normal.norm();
        normals.push_back(normal);
    }
    normals.push_back(Vector2d(0,0));
    return normals;
}

Vector2d icp_lidar_ceres::split_(string input, char delimiter) {
    Vector2d answer;
    stringstream ss(input);
    string temp;

    for(int i = 0; getline(ss, temp, delimiter); i++) {
        answer(i) = stod(temp);
    }
    return answer;
}

void icp_lidar_ceres::push_back_(MatrixXd& m, Vector2d&& values, std::size_t row) {
    if(row >= m.rows()) {
        m.conservativeResize(row + 1, Eigen::NoChange);
    }
    m.row(row) = values;
}

double** icp_lidar_ceres::pointsMatrix_to_2darray_(const MatrixXd& points) {
    double** array_2d = new double*[points.rows()];
    for(int i = 0; i < points.rows(); i++) {
        array_2d[i] = new double[2];
    }
    for(int i=0; i<points.rows(); i++) {
        for(int j=0; j<2; j++) {
            array_2d[i][j] = points(i,j);
        }
    }
    return array_2d;
}