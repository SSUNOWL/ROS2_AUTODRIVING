#ifndef MPC_CONTROLLER_MPC_SOLVER_HPP_
#define MPC_CONTROLLER_MPC_SOLVER_HPP_

#include "mpc_controller/dynamic_model.hpp"
#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <vector>

namespace mpc_controller
{

// [추가] 장애물 구조체
struct Obstacle {
    double x;
    double y;
    double r; // 반지름 (Safety Margin 포함)
};

class MPCSolver {
public:
    MPCSolver();
    ~MPCSolver();

    void init(int horizon, double dt);

    // [수정] 장애물 벡터를 인자로 받음
    Input solve(const State& current_state, 
                const std::vector<State>& ref_traj,
                const std::vector<Obstacle>& obstacles = {}); 

    std::vector<State> getPredictedTrajectory() const;

private:
    DynamicBicycleModel model_;
    OsqpEigen::Solver solver_;

    int N_;
    double dt_;

    // 차원 상수
    const int nx_ = 6; // State: x, y, psi, vx, vy, omega
    const int nu_ = 2; // Input: delta, acc
    const int ns_ = 1; // [추가] Slack variable (Soft Constraint용)

    Eigen::DiagonalMatrix<double, Eigen::Dynamic> Q_;
    Eigen::DiagonalMatrix<double, Eigen::Dynamic> R_;
    
    // [추가] Slack 변수 가중치
    double slack_weight_ = 10000.0; 

    std::vector<State> predicted_trajectory_;

    // [추가] 장애물 제약 선형화 헬퍼 함수
    void getObstacleConstraint(const State& ref_state,
                               const std::vector<Obstacle>& obstacles,
                               double& a_x, double& a_y, double& lower_bound);

    // [수정] QP Form 변환 함수 (Obstacles 추가)
    void castToQPForm(const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& Q,
                      const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& R,
                      const State& x0,
                      const std::vector<State>& ref,
                      const std::vector<Obstacle>& obstacles,
                      Eigen::SparseMatrix<double>& P_sparse,
                      Eigen::VectorXd& q,
                      Eigen::SparseMatrix<double>& A_sparse,
                      Eigen::VectorXd& l,
                      Eigen::VectorXd& u_vec);
};

} // namespace mpc_controller

#endif // MPC_CONTROLLER_MPC_SOLVER_HPP_