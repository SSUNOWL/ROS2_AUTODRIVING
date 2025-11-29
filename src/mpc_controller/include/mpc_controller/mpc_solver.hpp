#ifndef MPC_SOLVER_HPP
#define MPC_SOLVER_HPP

#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include <vector>
#include "mpc_controller/dynamic_model.hpp"

namespace mpc_controller
{

class MPCSolver {
public:
    MPCSolver();
    ~MPCSolver();

    // 초기화: 예측 구간(Horizon)과 시간 간격(dt) 설정
    void init(int horizon, double dt);

    // 핵심 함수: 현재 상태와 목표 경로를 받아 최적의 입력을 계산
    // reference_trajectory: 앞으로 N스텝 동안의 목표 상태들
    Input solve(const State& current_state, const std::vector<State>& reference_trajectory);

    std::vector<State> getPredictedTrajectory() const;
private:
    // MPC 파라미터
    int N_;          // 예측 구간 (Horizon Length)
    double dt_;      // 시간 간격
    
    // 차원 (State: 6, Input: 2)
    const int nx_ = 6; 
    const int nu_ = 2;

    // 가중치 행렬 (Diagonal Matrix)
    Eigen::DiagonalMatrix<double, 6> Q_; // 상태 오차 가중치
    Eigen::DiagonalMatrix<double, 2> R_; // 입력 사용량 가중치

    // OSQP Solver 인스턴스
    OsqpEigen::Solver solver_;

    // 차량 모델 인스턴스
    DynamicBicycleModel model_;

    // 내부 함수: QP 문제 생성을 위한 행렬 초기화 (Cast to QP form)
    void castToQPForm(const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& Q,
                      const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& R,
                      const State& current_state,
                      const std::vector<State>& ref_traj,
                      Eigen::SparseMatrix<double>& hessianMatrix,
                      Eigen::VectorXd& gradient,
                      Eigen::SparseMatrix<double>& linearMatrix,
                      Eigen::VectorXd& lowerBound,
                      Eigen::VectorXd& upperBound);

    std::vector<State> predicted_trajectory_;
};

} // namespace mpc_controller

#endif // MPC_SOLVER_HPP