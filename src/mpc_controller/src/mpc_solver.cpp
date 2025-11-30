#include "mpc_controller/mpc_solver.hpp"
#include <iostream>

namespace mpc_controller
{

MPCSolver::MPCSolver() {
    // 가중치 설정 (튜닝 포인트!)
    // x, y, psi, vx, vy, omega 순서
    // Q: 에러 허용치 (x, y, yaw에 집중)
    Q_.diagonal() << 2.0, 2.0, 5.0, 0.1, 0.0, 0.0;    // R (입력 가중치): "핸들 꺾는 건 싸니까 필요하면 팍팍 꺾어라!"
    // 아까 20000 이었던 것을 100 수준으로 대폭 낮춥니다.
    // 가속도(acc)도 낮춰서 시원하게 출발하게 합니다.
    R_.diagonal() << 10.0, 1.0;    
}

MPCSolver::~MPCSolver() {}

void MPCSolver::init(int horizon, double dt) {
    N_ = horizon;
    dt_ = dt;

    // OSQP Solver 설정
    solver_.settings()->setVerbosity(false);
    solver_.settings()->setWarmStart(true);
    
    // [변경] 여기서 변수 개수를 미리 세팅하지 않습니다. 
    // 실제 행렬이 만들어지는 solve() 시점에 정확한 크기를 넣는 게 안전합니다.
    solver_.data()->setNumberOfVariables(0);
    solver_.data()->setNumberOfConstraints(0);
}

Input MPCSolver::solve(const State& current_state, 
                       const std::vector<State>& ref_traj,
                       const std::vector<Obstacle>& obstacles) { // 추가됨    
    // 1. QP 문제 행렬 생성
    Eigen::SparseMatrix<double> P; 
    Eigen::VectorXd q;             
    Eigen::SparseMatrix<double> A_c; 
    Eigen::VectorXd l;             
    Eigen::VectorXd u;             

    castToQPForm(Q_, R_, current_state, ref_traj, obstacles, P, q, A_c, l, u);
    // 2. Solver 데이터 주입
    if (!solver_.isInitialized()) {
        // [핵심] 실패 후 재진입 시, 기존에 설정된 찌꺼기가 있다면 청소해줘야 함
        solver_.data()->clearHessianMatrix();
        solver_.data()->clearLinearConstraintsMatrix();

        // [핵심] 실제 생성된 행렬의 크기로 사이즈 설정 (Garbage Value 참조 방지)
        solver_.data()->setNumberOfVariables(P.rows());
        solver_.data()->setNumberOfConstraints(A_c.rows());

        if (!solver_.data()->setHessianMatrix(P)) return {};
        if (!solver_.data()->setGradient(q)) return {};
        if (!solver_.data()->setLinearConstraintsMatrix(A_c)) return {};
        if (!solver_.data()->setLowerBound(l)) return {};
        if (!solver_.data()->setUpperBound(u)) return {};
        
        if (!solver_.initSolver()) {
            std::cerr << "Solver init failed!" << std::endl;
            return {}; 
        }
    } else {
        // 이미 초기화되었다면 업데이트
        solver_.updateHessianMatrix(P);
        solver_.updateGradient(q);
        solver_.updateLinearConstraintsMatrix(A_c);
        solver_.updateBounds(l, u);
    }

    // 3. 문제 풀기
    if (solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        // 실패 시 로그만 띄우고 (차가 멈추지 않게) 이전 입력 유지하거나 0 반환
        std::cerr << "Solver failed to solve!" << std::endl;
        return {0.0, 0.0}; 
    }

    Eigen::VectorXd solution = solver_.getSolution();
    
    // 예측 궤적 저장
    predicted_trajectory_.clear();
    for (int k = 0; k <= N_; ++k) {
        int idx = k * (nx_ + nu_);
        if (idx + 5 < solution.size()) { // 인덱스 체크
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
                             const std::vector<Obstacle>& obstacles, // 추가됨
                             Eigen::SparseMatrix<double>& P_sparse,
                             Eigen::VectorXd& q,
                             Eigen::SparseMatrix<double>& A_sparse,
                             Eigen::VectorXd& l,
                             Eigen::VectorXd& u_vec)
{
    // 최적화 변수 벡터 z = [x0, u0, x1, u1, ..., xN]
    int num_vars = (nx_ + nu_) * N_ + nx_;
    int num_eq_constraints = nx_ * (N_ + 1); // 초기 상태 + 모델 등식 제약
    int num_ineq_constraints = nu_ * N_;     // 입력 제약 (핸들 꺾는 각도 제한 등)
    int num_obs_constraints = N_ + 1; 

    int num_constraints = num_eq_constraints + num_ineq_constraints + num_obs_constraints;  
    // --- 1. Hessian Matrix (P) & Gradient (q) 구성 ---
    // Cost J = 0.5 * z^T * P * z + q^T * z
    Eigen::VectorXd P_diag(num_vars);
    q = Eigen::VectorXd::Zero(num_vars);

    for (int k = 0; k < N_; ++k) {
        int idx = k * (nx_ + nu_);
        // 상태 오차 가중치
        P_diag.segment(idx, nx_) = Q.diagonal();
        // 입력 가중치
        P_diag.segment(idx + nx_, nu_) = R.diagonal();
        
        // Gradient q: -Q * x_ref (레퍼런스 추종을 위해 1차항 설정)
        // (x - x_ref)^T Q (x - x_ref) = x^T Q x - 2 x^T Q x_ref + ...
        // 여기서 -x^T Q x_ref 부분이 q^T x 가 됨 -> q = -Q * x_ref
        Eigen::VectorXd x_ref_vec(nx_);
        if (k < (int)ref.size()) 
            x_ref_vec << ref[k].x, ref[k].y, ref[k].psi, ref[k].vx, ref[k].vy, ref[k].omega;
        else
            x_ref_vec.setZero(); // 예외 처리

        q.segment(idx, nx_) = -Q.diagonal().cwiseProduct(x_ref_vec);
    }
    // 마지막 터미널 상태 가중치
    int final_idx = N_ * (nx_ + nu_);
    P_diag.segment(final_idx, nx_) = Q.diagonal(); 
    // 마지막 레퍼런스
    Eigen::VectorXd x_ref_final(nx_);
    if (!ref.empty()) {
        const auto& r = ref.back();
        x_ref_final << r.x, r.y, r.psi, r.vx, r.vy, r.omega;
    } else x_ref_final.setZero();
    q.segment(final_idx, nx_) = -Q.diagonal().cwiseProduct(x_ref_final);

    // P 행렬을 Sparse로 변환
    P_sparse = Eigen::SparseMatrix<double>(num_vars, num_vars);
    for(int i=0; i<num_vars; ++i) P_sparse.insert(i,i) = P_diag(i);


    // --- 2. Constraints (A_c, l, u) 구성 ---
    // 등식 제약: x_{k+1} = A_k * x_k + B_k * u_k
    // -> -A_k * x_k - B_k * u_k + x_{k+1} = 0
    
    // 행렬 채우기 (Triplet 사용 추천 - 속도 빠름)
    std::vector<Eigen::Triplet<double>> tripletList;

    l = Eigen::VectorXd::Zero(num_constraints);
    u_vec = Eigen::VectorXd::Zero(num_constraints);
    
    // [팁] 아주 작은 여유값 (Numerical Epsilon)
    // l = u 대신, l = val - eps, u = val + eps 로 설정하여 에러 방지
    double eps = 1e-4; 

    // 2-1. 초기 상태 제약 (x0 = current_state)
    for(int i=0; i<nx_; ++i) {
        tripletList.push_back(Eigen::Triplet<double>(i, i, 1.0));
        double val = 0;
        if(i==0) val=x0.x; else if(i==1) val=x0.y; else if(i==2) val=x0.psi;
        else if(i==3) val=x0.vx; else if(i==4) val=x0.vy; else if(i==5) val=x0.omega;
        
        // [수정] 여유 부여
        l(i) = val - eps; 
        u_vec(i) = val + eps;
    }

    // 2-2. 다이나믹스 제약 (Dynamics Constraints)
    for (int k = 0; k < N_; ++k) {
        Eigen::Matrix<double, 6, 6> A_k;
        Eigen::Matrix<double, 6, 2> B_k;
        
        // 기준 입력 u_zero
        Input u_zero; u_zero.delta = 0; u_zero.acc = 0;
        
        // 레퍼런스나 현재 상태 기반 선형화
        State lin_state = (k < (int)ref.size()) ? ref[k] : State(); 
        model_.getLinearizedMatrices(lin_state, u_zero, dt_, A_k, B_k);

        int row_start = nx_ * (k + 1);
        int col_x_k   = (nx_ + nu_) * k;
        int col_u_k   = col_x_k + nx_;
        int col_x_kp1 = (nx_ + nu_) * (k + 1);

        // x_{k+1}
        for(int i=0; i<nx_; ++i) 
            tripletList.push_back(Eigen::Triplet<double>(row_start + i, col_x_kp1 + i, 1.0));

        // -A x_k
        for(int r=0; r<nx_; ++r) {
            for(int c=0; c<nx_; ++c) {
                if(std::abs(A_k(r,c)) > 1e-5)
                    tripletList.push_back(Eigen::Triplet<double>(row_start + r, col_x_k + c, -A_k(r,c)));
            }
        }

        // -B u_k
        for(int r=0; r<nx_; ++r) {
            for(int c=0; c<nu_; ++c) {
                if(std::abs(B_k(r,c)) > 1e-5)
                    tripletList.push_back(Eigen::Triplet<double>(row_start + r, col_u_k + c, -B_k(r,c)));
            }
        }
        
        // [수정] 다이나믹스 등식 제약 (0 = 0) 에도 여유 부여
        // -A*x - B*u + x_next = 0
        l.segment(row_start, nx_).setConstant(-eps);
        u_vec.segment(row_start, nx_).setConstant(eps);
    }

    // 2-3. 입력 제약 (Inequality)
    int constraints_start = num_eq_constraints;
    double max_steer = 0.4189; 
    double max_acc = 5.0;

    for (int k = 0; k < N_; ++k) {
        int row = constraints_start + k * nu_;
        int col_u = k * (nx_ + nu_) + nx_;

        // Steering
        tripletList.push_back(Eigen::Triplet<double>(row, col_u, 1.0));
        l(row) = -max_steer;
        u_vec(row) = max_steer;

        // Acceleration
        tripletList.push_back(Eigen::Triplet<double>(row + 1, col_u + 1, 1.0));
        l(row + 1) = -max_acc;
        u_vec(row + 1) = max_acc;
    }
    //2-4 장애물 회피 제약
    
    int obs_start_idx = num_eq_constraints + num_ineq_constraints;
    double big_num = 1e10; // 무한대 대용 (OSQP_INFTY)

    for (int k = 0; k <= N_; ++k) {
        int row = obs_start_idx + k;
        int col_x = k * (nx_ + nu_);     // x 상태 변수 위치
        int col_y = k * (nx_ + nu_) + 1; // y 상태 변수 위치

        // 1. 현재 예측 위치(혹은 레퍼런스) 가져오기
        double ref_x = (k < (int)ref.size()) ? ref[k].x : ref.back().x;
        double ref_y = (k < (int)ref.size()) ? ref[k].y : ref.back().y;

        // 2. 가장 가까운 장애물 찾기
        const Obstacle* closest_obs = nullptr;
        double min_dist_sq = std::numeric_limits<double>::max();

        for (const auto& obs : obstacles) {
            double dx = ref_x - obs.x;
            double dy = ref_y - obs.y;
            double d_sq = dx*dx + dy*dy;
            if (d_sq < min_dist_sq) {
                min_dist_sq = d_sq;
                closest_obs = &obs;
            }
        }

        if (closest_obs != nullptr) {
            // 3. 선형화 계수 계산
            double dx = ref_x - closest_obs->x;
            double dy = ref_y - closest_obs->y;
            double dist = std::sqrt(min_dist_sq);
            
            // 0 나누기 방지
            if (dist < 1e-3) { dx = 1.0; dy = 0.0; dist = 1.0; }

            // A matrix 채우기: dx/dist * x + dy/dist * y
            // (로봇 위치 x, y에 곱해지는 계수)
            tripletList.push_back(Eigen::Triplet<double>(row, col_x, dx / dist));
            tripletList.push_back(Eigen::Triplet<double>(row, col_y, dy / dist));

            // 경계값 설정 (Lower Bound <= Ax <= Upper Bound)
            // Lower Bound: 안전거리 + (장애물 중심 보정)
            // 수식 유도: (x - xc)nx + (y - yc)ny >= r_safe
            // -> x*nx + y*ny >= r_safe + xc*nx + yc*ny
            
            double nx = dx / dist;
            double ny = dy / dist;
            
            l(row) = closest_obs->r + (closest_obs->x * nx + closest_obs->y * ny);
            u_vec(row) = big_num; // 상한선은 무한대 (멀어지는 건 OK)
        } else {
            // 장애물 없으면 제약조건 무효화 (0 * x >= -무한대)
            l(row) = -big_num;
            u_vec(row) = big_num;
        }
    }

    // Sparse Matrix 생성
    A_sparse = Eigen::SparseMatrix<double>(num_constraints, num_vars);
    A_sparse.setFromTriplets(tripletList.begin(), tripletList.end());
}

} // namespace mpc_controller