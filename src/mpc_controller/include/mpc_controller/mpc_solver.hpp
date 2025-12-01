#ifndef MPC_CONTROLLER_MPC_SOLVER_HPP_
#define MPC_CONTROLLER_MPC_SOLVER_HPP_

#include "mpc_controller/dynamic_model.hpp"
#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <vector>

namespace mpc_controller
{

class MPCSolver {
public:
    MPCSolver();
    ~MPCSolver();

    void init(int horizon, double dt);

    // [수정] 장애물(Obstacles) 인자 제거 -> 순수 경로 추종
    Input solve(const State& current_state, 
                const std::vector<State>& ref_traj); 

    std::vector<State> getPredictedTrajectory() const;

private:
    DynamicBicycleModel model_;
    OsqpEigen::Solver solver_;

    int N_;
    double dt_;

    // 차원 상수
    const int nx_ = 6; // State: x, y, psi, vx, vy, omega
    const int nu_ = 2; // Input: delta, acc
    // [제거] Slack variable (ns_) 삭제

    Eigen::DiagonalMatrix<double, Eigen::Dynamic> Q_;
    Eigen::DiagonalMatrix<double, Eigen::Dynamic> R_;
    
    // [제거] slack_weight_ 삭제

    std::vector<State> predicted_trajectory_;

    // [제거] getObstacleConstraint 삭제

    // [수정] QP Form 변환 함수 (Obstacles 인자 제거)
    void castToQPForm(const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& Q,
                      const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& R,
                      const State& x0,
                      const std::vector<State>& ref,
                      Eigen::SparseMatrix<double>& P_sparse,
                      Eigen::VectorXd& q,
                      Eigen::SparseMatrix<double>& A_sparse,
                      Eigen::VectorXd& l,
                      Eigen::VectorXd& u_vec);
};

} // namespace mpc_controller

#endif // MPC_CONTROLLER_MPC_SOLVER_HPP_