#include "mpc_controller/mpc_solver.hpp"
#include <iostream>
#include <limits>

namespace mpc_controller
{

MPCSolver::MPCSolver() {
    // [수정] Dynamic 행렬이므로, 값을 넣기 전에 반드시 크기를 지정해야 합니다!
    Q_.resize(nx_); // nx_ = 6
    R_.resize(nu_); // nu_ = 2

    // 가중치 설정 (튜닝 포인트!)
    // x, y, psi, vx, vy, omega 순서
    Q_.diagonal() << 2.0, 2.0, 1.0, 0.5, 0.0, 0.0;  
    // R (입력 가중치): 핸들을 팍팍 꺾을 수 있게 낮게 유지
    R_.diagonal() << 20.0, 1.0;    
    // [수정] Slack Weight 대폭 증가
    // 장애물 침범 시 벌점을 엄청나게 줘서 "절대 밟지 마"라고 경고
    slack_weight_ = 500000.0;
}

MPCSolver::~MPCSolver() {}

void MPCSolver::init(int horizon, double dt) {
    N_ = horizon;
    dt_ = dt;

    solver_.settings()->setVerbosity(false);
    solver_.settings()->setWarmStart(true);
    
    solver_.data()->setNumberOfVariables(0);
    solver_.data()->setNumberOfConstraints(0);
}

// [수정] 장애물 리스트를 받는 solve 함수
Input MPCSolver::solve(const State& current_state, 
                       const std::vector<State>& ref_traj,
                       const std::vector<Obstacle>& obstacles) {
    
    // 1. QP 문제 행렬 생성
    Eigen::SparseMatrix<double> P; 
    Eigen::VectorXd q;             
    Eigen::SparseMatrix<double> A_c; 
    Eigen::VectorXd l;             
    Eigen::VectorXd u;             

    // 장애물 정보를 포함하여 행렬 생성
    castToQPForm(Q_, R_, current_state, ref_traj, obstacles, P, q, A_c, l, u);

    // [핵심 수정] 
    // 이미 Solver가 초기화되어 있다면 메모리를 해제해야 재설정(Init)이 가능합니다.
    if (solver_.isInitialized()) {
        solver_.clearSolver();
    }

    // 기존 데이터 홀더 클리어
    solver_.data()->clearHessianMatrix();
    solver_.data()->clearLinearConstraintsMatrix();

    // 변수/제약 개수 재설정 (장애물 유무에 따라 크기가 변할 수 있음)
    solver_.data()->setNumberOfVariables(P.rows());
    solver_.data()->setNumberOfConstraints(A_c.rows());

    // 데이터 주입
    if (!solver_.data()->setHessianMatrix(P)) return {0.0, 0.0};
    if (!solver_.data()->setGradient(q)) return {0.0, 0.0};
    if (!solver_.data()->setLinearConstraintsMatrix(A_c)) return {0.0, 0.0};
    if (!solver_.data()->setLowerBound(l)) return {0.0, 0.0};
    if (!solver_.data()->setUpperBound(u)) return {0.0, 0.0};
    
    // Solver 초기화 및 실행
    // 매번 구조를 새로 짜므로 WarmStart는 끄는 것이 안전합니다.
    solver_.settings()->setWarmStart(false); 
    
    if (!solver_.initSolver()) {
        std::cerr << "Solver init failed!" << std::endl;
        return {0.0, 0.0}; 
    }

    // 3. 문제 풀기
    if (solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        // std::cerr << "Solver failed to solve!" << std::endl;
        return {0.0, 0.0}; 
    }

    Eigen::VectorXd solution = solver_.getSolution();
    
    // 변수 구조: nx(6) + nu(2) + ns(1) = 9
    int dim = nx_ + nu_ + ns_;

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

// [신규] 장애물 제약 선형화 (Linearization)
void MPCSolver::getObstacleConstraint(const State& ref_state,
                                      const std::vector<Obstacle>& obstacles,
                                      double& a_x, double& a_y, double& lower_bound)
{
    // 기본값: 제약 없음 (0*x + 0*y >= -Infinite)
    a_x = 0.0; a_y = 0.0; lower_bound = -1e19;

    if (obstacles.empty()) return;

    // 1. 가장 가까운 장애물 찾기
    int closest_idx = -1;
    double min_dist_sq = std::numeric_limits<double>::max();

    for (size_t i = 0; i < obstacles.size(); ++i) {
        double dx = ref_state.x - obstacles[i].x;
        double dy = ref_state.y - obstacles[i].y;
        double d2 = dx*dx + dy*dy;
        if (d2 < min_dist_sq) {
            min_dist_sq = d2;
            closest_idx = i;
        }
    }

    if (closest_idx == -1) return;
    const auto& obs = obstacles[closest_idx];
    double dist = std::sqrt(min_dist_sq);
    
    // 최적화 효율을 위해 너무 먼 장애물은 무시 (반지름 + 1.5m)
    if (dist > obs.r + 1.5) return; 
    
    // 분모 0 방지
    if (dist < 1e-4) dist = 1e-4;

    // 2. 선형화 계수 계산 (Hyperplane 생성)
    // 장애물 중심 -> 로봇(Ref) 방향 벡터가 법선 벡터(Normal Vector)
    double nx = (ref_state.x - obs.x) / dist;
    double ny = (ref_state.y - obs.y) / dist;

    a_x = nx;
    a_y = ny;
    
    // 부등식: nx * x + ny * y >= (nx * obs_x + ny * obs_y) + radius
    // 즉, 장애물 중심에서 법선 방향으로 반지름만큼 떨어진 평면 바깥에 있어야 함
    lower_bound = (nx * obs.x + ny * obs.y) + obs.r;
}

void MPCSolver::castToQPForm(const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& Q,
                             const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& R,
                             const State& x0,
                             const std::vector<State>& ref,
                             const std::vector<Obstacle>& obstacles,
                             Eigen::SparseMatrix<double>& P_sparse,
                             Eigen::VectorXd& q,
                             Eigen::SparseMatrix<double>& A_sparse,
                             Eigen::VectorXd& l,
                             Eigen::VectorXd& u_vec)
{
    // --------------------------------------------------------
    // Step 1: 변수 크기 및 인덱스 설정
    // --------------------------------------------------------
    // z_k = [x_k, u_k, epsilon_k] 
    // 크기: 6 + 2 + 1 = 9
    int dim = nx_ + nu_ + ns_;
    
    // 전체 변수: (N step * dim) + 마지막 State(nx)
    int num_vars = dim * N_ + nx_;

    // 제약 조건 개수
    int num_eq_constraints = nx_ * (N_ + 1); // Dynamics (x0 포함)
    int num_ineq_constraints = nu_ * N_;     // Input Limits
    int num_slack_constraints = ns_ * N_;    // Slack >= 0 (Non-negative)
    int num_obs_constraints = N_;            // Obstacle Avoidance (Step 당 1개)

    int num_constraints = num_eq_constraints + num_ineq_constraints 
                        + num_slack_constraints + num_obs_constraints;

    // --------------------------------------------------------
    // Step 2: Hessian (P) & Gradient (q) 구성
    // Cost J = 0.5 * z^T * P * z + q^T * z
    // --------------------------------------------------------
    std::vector<Eigen::Triplet<double>> p_triplets;
    q = Eigen::VectorXd::Zero(num_vars);

    for (int k = 0; k < N_; ++k) {
        int idx = k * dim;
        
        // 1) State Cost (Q)
        for (int i = 0; i < nx_; ++i) {
            p_triplets.push_back({idx + i, idx + i, Q.diagonal()[i]});
        }
        // 2) Input Cost (R)
        for (int i = 0; i < nu_; ++i) {
            p_triplets.push_back({idx + nx_ + i, idx + nx_ + i, R.diagonal()[i]});
        }
        // 3) Slack Cost (Weight)
        // Soft Constraint: epsilon^2 항 추가
        p_triplets.push_back({idx + nx_ + nu_, idx + nx_ + nu_, slack_weight_});

        // 4) Gradient q (Reference Tracking)
        // q = -Q * x_ref
        Eigen::VectorXd x_ref_vec(nx_);
        if (k < (int)ref.size()) 
            x_ref_vec << ref[k].x, ref[k].y, ref[k].psi, ref[k].vx, ref[k].vy, ref[k].omega;
        else x_ref_vec.setZero();

        Eigen::VectorXd q_k = -Q.diagonal().cwiseProduct(x_ref_vec);
        for(int i=0; i<nx_; ++i) q(idx + i) = q_k(i);
        
        // Slack 변수의 Gradient는 0 (제곱항만 최소화)
    }

    // Terminal Cost (마지막 점은 State만 존재)
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

    // --------------------------------------------------------
    // Step 3: Constraints (A, l, u) 구성
    // --------------------------------------------------------
    std::vector<Eigen::Triplet<double>> tripletList;
    l = Eigen::VectorXd::Zero(num_constraints);
    u_vec = Eigen::VectorXd::Zero(num_constraints);
    
    double eps = 1e-4; // Numerical stability

    // --- 3-1. Initial State Constraint ---
    for(int i=0; i<nx_; ++i) {
        tripletList.push_back({i, i, 1.0});
        double val = 0;
        if(i==0) val=x0.x; else if(i==1) val=x0.y; else if(i==2) val=x0.psi;
        else if(i==3) val=x0.vx; else if(i==4) val=x0.vy; else if(i==5) val=x0.omega;
        
        l(i) = val - eps; 
        u_vec(i) = val + eps;
    }

    // --- 3-2. Dynamics Constraints ---
    // x_{k+1} = A x_k + B u_k
    // -> -A x_k - B u_k + x_{k+1} = 0
    for (int k = 0; k < N_; ++k) {
        Eigen::Matrix<double, 6, 6> A_k;
        Eigen::Matrix<double, 6, 2> B_k;
        
        Input u_zero; u_zero.delta = 0; u_zero.acc = 0;
        State lin_state = (k < (int)ref.size()) ? ref[k] : State(); 
        model_.getLinearizedMatrices(lin_state, u_zero, dt_, A_k, B_k);

        int row_start = nx_ * (k + 1);
        int col_x_k   = k * dim;        // x_k 위치
        int col_u_k   = col_x_k + nx_;  // u_k 위치
        int col_x_kp1 = (k + 1) * dim;  // x_{k+1} 위치 (다음 스텝의 state 시작점)
        
        // 마지막 스텝의 경우 x_{k+1}은 변수 벡터 끝부분
        if (k == N_ - 1) col_x_kp1 = N_ * dim;

        // x_{k+1} (Identity)
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
        
        // Slack 변수는 Dynamics에 영향을 주지 않으므로 계수 0 (추가 안 함)

        l.segment(row_start, nx_).setConstant(-eps);
        u_vec.segment(row_start, nx_).setConstant(eps);
    }

    // --- 3-3. Input Constraints ---
    int constr_idx = num_eq_constraints;
    double max_steer = 0.4189; 
    double max_acc = 5.0;

    for (int k = 0; k < N_; ++k) {
        int col_u = k * dim + nx_; // u_k 시작 위치
        
        // Delta
        tripletList.push_back({constr_idx, col_u, 1.0});
        l(constr_idx) = -max_steer; u_vec(constr_idx) = max_steer;
        constr_idx++;

        // Accel
        tripletList.push_back({constr_idx, col_u + 1, 1.0});
        l(constr_idx) = -max_acc; u_vec(constr_idx) = max_acc;
        constr_idx++;
    }

    // --- 3-4. Slack Variable Constraints (Non-negative) ---
    // epsilon >= 0
    for (int k = 0; k < N_; ++k) {
        int col_slack = k * dim + nx_ + nu_; // slack 위치
        
        tripletList.push_back({constr_idx, col_slack, 1.0});
        l(constr_idx) = 0.0; 
        u_vec(constr_idx) = 2e19; // Infinity
        constr_idx++;
    }

    // --- 3-5. Obstacle Avoidance Constraints (Soft) ---
    // a_x * x + a_y * y + 1.0 * epsilon >= lower_bound
    for (int k = 0; k < N_; ++k) {
        State ref_s = (k < (int)ref.size()) ? ref[k] : State();
        double a_x, a_y, lower_val;
        
        // 헬퍼 함수 호출
        getObstacleConstraint(ref_s, obstacles, a_x, a_y, lower_val);

        int col_x = k * dim;
        int col_y = k * dim + 1;
        int col_slack = k * dim + nx_ + nu_;

        if (std::abs(a_x) > 1e-5 || std::abs(a_y) > 1e-5) {
            tripletList.push_back({constr_idx, col_x, a_x});
            tripletList.push_back({constr_idx, col_y, a_y});
            // [중요] Slack 변수를 제약식에 포함시켜 위반 허용
            tripletList.push_back({constr_idx, col_slack, 1.0});

            l(constr_idx) = lower_val;
            u_vec(constr_idx) = 2e19; // 상한 없음
        } else {
            // 장애물 없으면 Constraint 무효화
            l(constr_idx) = -2e19;
            u_vec(constr_idx) = 2e19;
        }
        constr_idx++;
    }

    A_sparse = Eigen::SparseMatrix<double>(num_constraints, num_vars);
    A_sparse.setFromTriplets(tripletList.begin(), tripletList.end());
}

} // namespace mpc_controller