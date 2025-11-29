#include "mpc_controller/dynamic_model.hpp"

namespace mpc_controller
{

DynamicBicycleModel::DynamicBicycleModel() {
    // 기본 파라미터 초기화
}

DynamicBicycleModel::~DynamicBicycleModel() {}

void DynamicBicycleModel::setParams(const VehicleParams& params) {
    params_ = params;
}

State DynamicBicycleModel::predict(const State& s, const Input& u, double dt) {
    State next_state;

    double m = params_.m;
    double L_f = params_.L_f;
    double L_r = params_.L_r;
    double I_z = params_.I_z;
    double C_f = params_.C_f;
    double C_r = params_.C_r;

    double vx_safe = (std::abs(s.vx) < 0.1) ? 0.1 : s.vx; 

    // 1. 슬립 앵글 (Slip Angle, alpha) 계산
    // [삭제됨] 이전 답변의 이미지 태그가 있던 자리입니다.
    double alpha_f = u.delta - std::atan((s.vy + L_f * s.omega) / vx_safe);
    double alpha_r = -std::atan((s.vy - L_r * s.omega) / vx_safe);

    // 2. 타이어 횡력 (Lateral Force, Fy) 계산
    double F_yf = C_f * alpha_f;
    double F_yr = C_r * alpha_r;

    // 3. 운동 방정식 (Dynamic Bicycle Model Equations)
    double vx_dot = u.acc - (F_yf * std::sin(u.delta)) / m + s.vy * s.omega;
    double vy_dot = (F_yf * std::cos(u.delta) + F_yr) / m - s.vx * s.omega;
    double omega_dot = (L_f * F_yf * std::cos(u.delta) - L_r * F_yr) / I_z;

    // 4. 상태 업데이트
    double x_dot = s.vx * std::cos(s.psi) - s.vy * std::sin(s.psi);
    double y_dot = s.vx * std::sin(s.psi) + s.vy * std::cos(s.psi);

    next_state.x = s.x + x_dot * dt;
    next_state.y = s.y + y_dot * dt;
    next_state.psi = s.psi + s.omega * dt;
    next_state.vx = s.vx + vx_dot * dt;
    next_state.vy = s.vy + vy_dot * dt;
    next_state.omega = s.omega + omega_dot * dt;

    return next_state;
}

void DynamicBicycleModel::getLinearizedMatrices(const State& s, const Input& u, double dt,
                                                Eigen::Matrix<double, 6, 6>& A,
                                                Eigen::Matrix<double, 6, 2>& B)
{
    // 아주 작은 변화량 (Epsilon)
    double eps = 1e-5;

    // 기준점 (현재 상태에서의 다음 상태)
    State s_next_nominal = predict(s, u, dt);

    // 1. A 행렬 계산 (Jacobian w.r.t State)
    // State 변수가 6개이므로 6번 반복
    // x, y, psi, vx, vy, omega 순서
    
    // 상태를 벡터로 변환하기 위한 람다/헬퍼가 있으면 좋지만, 여기선 직관적으로 풉니다.
    // 임시 상태 변수들
    State s_perturb; 
    
    // -- 1열: dx/dx --
    s_perturb = s; s_perturb.x += eps;
    State s_next = predict(s_perturb, u, dt);
    A(0,0) = (s_next.x - s_next_nominal.x) / eps; A(1,0) = (s_next.y - s_next_nominal.y) / eps; 
    A(2,0) = (s_next.psi - s_next_nominal.psi) / eps; A(3,0) = (s_next.vx - s_next_nominal.vx) / eps;
    A(4,0) = (s_next.vy - s_next_nominal.vy) / eps; A(5,0) = (s_next.omega - s_next_nominal.omega) / eps;

    // -- 2열: dx/dy --
    s_perturb = s; s_perturb.y += eps;
    s_next = predict(s_perturb, u, dt);
    A(0,1) = (s_next.x - s_next_nominal.x) / eps; A(1,1) = (s_next.y - s_next_nominal.y) / eps;
    A(2,1) = (s_next.psi - s_next_nominal.psi) / eps; A(3,1) = (s_next.vx - s_next_nominal.vx) / eps;
    A(4,1) = (s_next.vy - s_next_nominal.vy) / eps; A(5,1) = (s_next.omega - s_next_nominal.omega) / eps;

    // -- 3열: dx/dpsi --
    s_perturb = s; s_perturb.psi += eps;
    s_next = predict(s_perturb, u, dt);
    A(0,2) = (s_next.x - s_next_nominal.x) / eps; A(1,2) = (s_next.y - s_next_nominal.y) / eps;
    A(2,2) = (s_next.psi - s_next_nominal.psi) / eps; A(3,2) = (s_next.vx - s_next_nominal.vx) / eps;
    A(4,2) = (s_next.vy - s_next_nominal.vy) / eps; A(5,2) = (s_next.omega - s_next_nominal.omega) / eps;

    // -- 4열: dx/dvx --
    s_perturb = s; s_perturb.vx += eps;
    s_next = predict(s_perturb, u, dt);
    A(0,3) = (s_next.x - s_next_nominal.x) / eps; A(1,3) = (s_next.y - s_next_nominal.y) / eps;
    A(2,3) = (s_next.psi - s_next_nominal.psi) / eps; A(3,3) = (s_next.vx - s_next_nominal.vx) / eps;
    A(4,3) = (s_next.vy - s_next_nominal.vy) / eps; A(5,3) = (s_next.omega - s_next_nominal.omega) / eps;

    // -- 5열: dx/dvy --
    s_perturb = s; s_perturb.vy += eps;
    s_next = predict(s_perturb, u, dt);
    A(0,4) = (s_next.x - s_next_nominal.x) / eps; A(1,4) = (s_next.y - s_next_nominal.y) / eps;
    A(2,4) = (s_next.psi - s_next_nominal.psi) / eps; A(3,4) = (s_next.vx - s_next_nominal.vx) / eps;
    A(4,4) = (s_next.vy - s_next_nominal.vy) / eps; A(5,4) = (s_next.omega - s_next_nominal.omega) / eps;

    // -- 6열: dx/domega --
    s_perturb = s; s_perturb.omega += eps;
    s_next = predict(s_perturb, u, dt);
    A(0,5) = (s_next.x - s_next_nominal.x) / eps; A(1,5) = (s_next.y - s_next_nominal.y) / eps;
    A(2,5) = (s_next.psi - s_next_nominal.psi) / eps; A(3,5) = (s_next.vx - s_next_nominal.vx) / eps;
    A(4,5) = (s_next.vy - s_next_nominal.vy) / eps; A(5,5) = (s_next.omega - s_next_nominal.omega) / eps;


    // 2. B 행렬 계산 (Jacobian w.r.t Input)
    Input u_perturb;

    // -- 1열: dx/ddelta (조향각) --
    u_perturb = u; u_perturb.delta += eps;
    s_next = predict(s, u_perturb, dt);
    B(0,0) = (s_next.x - s_next_nominal.x) / eps; B(1,0) = (s_next.y - s_next_nominal.y) / eps;
    B(2,0) = (s_next.psi - s_next_nominal.psi) / eps; B(3,0) = (s_next.vx - s_next_nominal.vx) / eps;
    B(4,0) = (s_next.vy - s_next_nominal.vy) / eps; B(5,0) = (s_next.omega - s_next_nominal.omega) / eps;

    // -- 2열: dx/dacc (가속도) --
    u_perturb = u; u_perturb.acc += eps;
    s_next = predict(s, u_perturb, dt);
    B(0,1) = (s_next.x - s_next_nominal.x) / eps; B(1,1) = (s_next.y - s_next_nominal.y) / eps;
    B(2,1) = (s_next.psi - s_next_nominal.psi) / eps; B(3,1) = (s_next.vx - s_next_nominal.vx) / eps;
    B(4,1) = (s_next.vy - s_next_nominal.vy) / eps; B(5,1) = (s_next.omega - s_next_nominal.omega) / eps;
}

} // namespace mpc_controller