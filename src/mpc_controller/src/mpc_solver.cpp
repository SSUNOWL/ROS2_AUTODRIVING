#include "mpc_controller/mpc_solver.hpp"
#include <iostream>
#include <limits>

namespace mpc_controller
{

MPCSolver::MPCSolver() {
    Q_.resize(nx_); // nx_ = 6
    R_.resize(nu_); // nu_ = 2

    // 1. Q (State Weights) - 경로 추종 전용 가중치
    // 장애물 회피 부담이 없으므로 경로 추종(x, y, psi)에 집중합니다.
    Q_.diagonal() << 5.0, 5.0, 3.0, 1.0, 0.0, 0.0; // [변경] 고속 레이싱을 위해 경로 추종 및 속도 가중치 증가
    
    // 2. R (Input Weights) - 부드러운 주행 유도
    R_.diagonal() << 5.0, 2.0; // [변경] 조향각 가중치 감소(덜 보수적) 및 가속도 가중치 증가
}

MPCSolver::~MPCSolver() {}

void MPCSolver::init(int horizon, double dt) {
    N_ = horizon;
    dt_ = dt;

    solver_.settings()->setVerbosity(false);
    solver_.settings()->setWarmStart(true);
    
    // 초기화 시 0으로 설정
    solver_.data()->setNumberOfVariables(0);
    solver_.data()->setNumberOfConstraints(0);
}

// [수정] 장애물 인자 제거됨
Input MPCSolver::solve(const State& current_state, 
                       const std::vector<State>& ref_traj) {
    
    // 1. QP 문제 행렬 생성
    Eigen::SparseMatrix<double> P; 
    Eigen::VectorXd q;             
    Eigen::SparseMatrix<double> A_c; 
    Eigen::VectorXd l;             
    Eigen::VectorXd u;             

    // [수정] 장애물 없이 순수 경로 추종 문제로 변환
    castToQPForm(Q_, R_, current_state, ref_traj, P, q, A_c, l, u);

    // Solver 재설정 로직 (행렬 크기가 고정되므로 초기화 최적화 가능하지만 안전하게 매번 클리어)
    if (solver_.isInitialized()) {
        solver_.clearSolver();
    }

    solver_.data()->clearHessianMatrix();
    solver_.data()->clearLinearConstraintsMatrix();

    solver_.data()->setNumberOfVariables(P.rows());
    solver_.data()->setNumberOfConstraints(A_c.rows());

    if (!solver_.data()->setHessianMatrix(P)) return {0.0, 0.0};
    if (!solver_.data()->setGradient(q)) return {0.0, 0.0};
    if (!solver_.data()->setLinearConstraintsMatrix(A_c)) return {0.0, 0.0};
    if (!solver_.data()->setLowerBound(l)) return {0.0, 0.0};
    if (!solver_.data()->setUpperBound(u)) return {0.0, 0.0};
    
    solver_.settings()->setWarmStart(false); 
    
    if (!solver_.initSolver()) {
        std::cerr << "Solver init failed!" << std::endl;
        return {0.0, 0.0}; 
    }

    if (solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        return {0.0, 0.0}; 
    }

    Eigen::VectorXd solution = solver_.getSolution();
    
    // [수정] 변수 구조: nx(6) + nu(2) = 8 (Slack 없음)
    int dim = nx_ + nu_;

    predicted_trajectory_.clear();
    for (int k = 0; k <= N_; ++k) {
        int idx = k * dim; 
        if (idx + 5 < solution.size()) {
            State s;
            s.x = solution(idx);
            s.y = solution(idx + 1);
            s.psi = solution(idx + 2);
            s.vx = solution(idx + 3);
            s.vy = solution(idx + 4);
            s.omega = solution(idx + 5);
            predicted_trajectory_.push_back(s);
        }
    }

    Input optimal_input;
    if (solution.size() >= nx_ + 2) {
        optimal_input.delta = solution(nx_);     
        optimal_input.acc   = solution(nx_ + 1); 
    } else {
        optimal_input = {0.0, 0.0};
    }

    return optimal_input;
}

std::vector<State> MPCSolver::getPredictedTrajectory() const {
    return predicted_trajectory_;
}

void MPCSolver::castToQPForm(const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& Q,
                             const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& R,
                             const State& x0,
                             const std::vector<State>& ref,
                             Eigen::SparseMatrix<double>& P_sparse,
                             Eigen::VectorXd& q,
                             Eigen::SparseMatrix<double>& A_sparse,
                             Eigen::VectorXd& l,
                             Eigen::VectorXd& u_vec)
{
    // [수정] Slack 변수가 없으므로 차원 축소
    // z_k = [x_k, u_k] -> 크기: 6 + 2 = 8
    int dim = nx_ + nu_;
    
    int num_vars = dim * N_ + nx_;

    // 제약 조건 개수 (장애물 제약 삭제됨)
    int num_eq_constraints = nx_ * (N_ + 1); // Dynamics
    int num_ineq_constraints = nu_ * N_;     // Input Limits

    int num_constraints = num_eq_constraints + num_ineq_constraints;

    // --- 1. Hessian (P) & Gradient (q) ---
    std::vector<Eigen::Triplet<double>> p_triplets;
    q = Eigen::VectorXd::Zero(num_vars);

    for (int k = 0; k < N_; ++k) {
        int idx = k * dim;
        
        // State Cost (Q)
        for (int i = 0; i < nx_; ++i) {
            p_triplets.push_back({idx + i, idx + i, Q.diagonal()[i]});
        }
        // Input Cost (R)
        for (int i = 0; i < nu_; ++i) {
            p_triplets.push_back({idx + nx_ + i, idx + nx_ + i, R.diagonal()[i]});
        }
        // Slack Cost 삭제됨

        // Gradient q (Reference Tracking)
        Eigen::VectorXd x_ref_vec(nx_);
        if (k < (int)ref.size()) 
            x_ref_vec << ref[k].x, ref[k].y, ref[k].psi, ref[k].vx, ref[k].vy, ref[k].omega;
        else x_ref_vec.setZero();

        Eigen::VectorXd q_k = -Q.diagonal().cwiseProduct(x_ref_vec);
        for(int i=0; i<nx_; ++i) q(idx + i) = q_k(i);
    }

    // Terminal Cost
    int final_idx = N_ * dim;
    for (int i = 0; i < nx_; ++i) {
        p_triplets.push_back({final_idx + i, final_idx + i, Q.diagonal()[i]});
    }
    // Terminal Reference
    Eigen::VectorXd x_ref_final(nx_);
    if (!ref.empty()) {
        const auto& r = ref.back();
        x_ref_final << r.x, r.y, r.psi, r.vx, r.vy, r.omega;
    } else x_ref_final.setZero();
    Eigen::VectorXd q_final = -Q.diagonal().cwiseProduct(x_ref_final);
    for(int i=0; i<nx_; ++i) q(final_idx + i) = q_final(i);

    P_sparse = Eigen::SparseMatrix<double>(num_vars, num_vars);
    P_sparse.setFromTriplets(p_triplets.begin(), p_triplets.end());

    // --- 2. Constraints (A, l, u) ---
    std::vector<Eigen::Triplet<double>> tripletList;
    l = Eigen::VectorXd::Zero(num_constraints);
    u_vec = Eigen::VectorXd::Zero(num_constraints);
    
    double eps = 1e-4;

    // 2-1. Initial State
    for(int i=0; i<nx_; ++i) {
        tripletList.push_back({i, i, 1.0});
        double val = 0;
        if(i==0) val=x0.x; else if(i==1) val=x0.y; else if(i==2) val=x0.psi;
        else if(i==3) val=x0.vx; else if(i==4) val=x0.vy; else if(i==5) val=x0.omega;
        
        l(i) = val - eps; 
        u_vec(i) = val + eps;
    }

    // 2-2. Dynamics
    for (int k = 0; k < N_; ++k) {
        Eigen::Matrix<double, 6, 6> A_k;
        Eigen::Matrix<double, 6, 2> B_k;
        
        Input u_zero; u_zero.delta = 0; u_zero.acc = 0;
        State lin_state = (k < (int)ref.size()) ? ref[k] : State(); 
        model_.getLinearizedMatrices(lin_state, u_zero, dt_, A_k, B_k);

        int row_start = nx_ * (k + 1);
        int col_x_k   = k * dim;
        int col_u_k   = col_x_k + nx_;
        int col_x_kp1 = (k + 1) * dim;
        
        if (k == N_ - 1) col_x_kp1 = N_ * dim;

        // x_{k+1}
        for(int i=0; i<nx_; ++i) 
            tripletList.push_back({row_start + i, col_x_kp1 + i, 1.0});

        // -A x_k
        for(int r=0; r<nx_; ++r) {
            for(int c=0; c<nx_; ++c) {
                if(std::abs(A_k(r,c)) > 1e-5)
                    tripletList.push_back({row_start + r, col_x_k + c, -A_k(r,c)});
            }
        }

        // -B u_k
        for(int r=0; r<nx_; ++r) {
            for(int c=0; c<nu_; ++c) {
                if(std::abs(B_k(r,c)) > 1e-5)
                    tripletList.push_back({row_start + r, col_u_k + c, -B_k(r,c)});
            }
        }

        l.segment(row_start, nx_).setConstant(-eps);
        u_vec.segment(row_start, nx_).setConstant(eps);
    }

    // 2-3. Input Constraints
    int constr_idx = num_eq_constraints;
    double max_steer = 0.4189; 
    double max_acc = 8.0;

    for (int k = 0; k < N_; ++k) {
        int col_u = k * dim + nx_;
        
        // Delta
        tripletList.push_back({constr_idx, col_u, 1.0});
        l(constr_idx) = -max_steer; u_vec(constr_idx) = max_steer;
        constr_idx++;

        // Accel
        tripletList.push_back({constr_idx, col_u + 1, 1.0});
        l(constr_idx) = -max_acc; u_vec(constr_idx) = max_acc;
        constr_idx++;
    }

    // [제거] Slack Constraints 삭제됨
    // [제거] Obstacle Avoidance Constraints 삭제됨

    A_sparse = Eigen::SparseMatrix<double>(num_constraints, num_vars);
    A_sparse.setFromTriplets(tripletList.begin(), tripletList.end());
}

} // namespace mpc_controller