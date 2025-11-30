#ifndef DYNAMIC_MODEL_HPP
#define DYNAMIC_MODEL_HPP

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

namespace mpc_controller
{

// 1. 차량 파라미터 구조체 (F1TENTH 표준 값 참조)
struct VehicleParams {
    double m = 2.434;       // 차량 질량 (kg)
    double L_f = 0.15875;   // 무게중심에서 앞바퀴까지 거리 (m)
    double L_r = 0.17145;   // 무게중심에서 뒷바퀴까지 거리 (m)
    double I_z = 0.04712;   // 관성 모멘트 (kg*m^2) - 차량 회전 저항성
    double C_f = 2.3;       // 앞바퀴 코너링 강성 (N/rad) - 타이어 특성 (중요!)
    double C_r = 2.3;       // 뒷바퀴 코너링 강성 (N/rad)
};

// 2. 상태 변수 (State): [x, y, psi, vx, vy, omega] (6개)
struct State {
    double x = 0.0;
    double y = 0.0;
    double psi = 0.0;   // 헤딩 (Yaw)
    double vx = 0.0;    // 차량 길이 방향 속도
    double vy = 0.0;    // 차량 측면 방향 속도 (Dynamic 모델의 핵심)
    double omega = 0.0; // 각속도 (Yaw rate)
};

// 3. 제어 입력 (Control Input): [steering_angle, acceleration]
struct Input {
    double delta = 0.0; // 조향각 (rad)
    double acc = 0.0;   // 가속도 (m/s^2)
};

class DynamicBicycleModel {
public:
    DynamicBicycleModel();
    ~DynamicBicycleModel();

    // 파라미터 설정 함수
    void setParams(const VehicleParams& params);

    // 핵심 함수: 현재 상태와 입력을 받아 다음 상태를 계산 (dt 후의 상태)
    State predict(const State& current_state, const Input& input, double dt);

    void getLinearizedMatrices(const State& s, const Input& u, double dt,
                               Eigen::Matrix<double, 6, 6>& A,
                               Eigen::Matrix<double, 6, 2>& B);
private:
    VehicleParams params_;
};

} // namespace mpc_controller

#endif // DYNAMIC_MODEL_HPP