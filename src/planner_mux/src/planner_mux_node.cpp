#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <limits>

using std::placeholders::_1;

struct Metrics
{
  double min_obs_dist;     // (기하학적) 최소 여유 거리 [m]
  double avg_speed;        // 평균 속도 [m/s]
  double jerk_cost;        // 속도 프로파일의 jerk 비용
  double tracking_error;   // 글로벌 경로 추종 오차
  double risk_index;       // 기존 위험 지수 (속도*가까운 장애물 기반)

  double max_lat_accel;    // 경로 전체에서의 최대 횡가속도 [m/s^2]
  double dyn_clear_margin; // min(d_eff - d_dyn(v)) : 동역학 안전거리 여유
};

struct Point2D
{
  double x;
  double y;
};

class LocalPlannerMux : public rclcpp::Node
{
public:
  LocalPlannerMux() : Node("local_planner_mux")
  {
    // ------- 파라미터 선언 & 로드 -------

    // 최소 기하학적 안전거리 (차량 외곽 기준이 아니라 "여유" 기준) [m]
    d_min_      = this->declare_parameter("d_min", 0.30);

    // 정규화 기준 값
    v_ref_      = this->declare_parameter("v_ref", 5.0);
    jerk_ref_   = this->declare_parameter("jerk_ref", 5.0);
    track_ref_  = this->declare_parameter("track_ref", 0.5);

    // 효율 모드 가중치
    w_speed_    = this->declare_parameter("w_speed",   1.0);
    w_track_    = this->declare_parameter("w_track",   1.0);
    w_comfort_  = this->declare_parameter("w_comfort", 1.0);

    // 차량 크기 및 여유 거리/위험도 기준
    vehicle_radius_ = this->declare_parameter("vehicle_radius", 0.25);
    clearance_ref_  = this->declare_parameter("clearance_ref", 0.5);
    risk_ref_       = this->declare_parameter("risk_ref", 1.0);
    w_clearance_    = this->declare_parameter("w_clearance", 1.0);

    // 동역학 관련 파라미터 (F1TENTH에 맞게 튜닝)
    // 최대 허용 횡가속도 (마찰계수*중력 ~ 7~8 m/s^2 근처)
    a_lat_max_   = this->declare_parameter("a_lat_max", 7.0);
    // 최대 제동 감속도 (절대값) [m/s^2]
    a_brake_max_ = this->declare_parameter("a_brake_max", 8.0);
    // 반응 시간 [s] (컨트롤/플래너 지연 포함)
    react_time_  = this->declare_parameter("react_time", 0.3);
    // 동역학 관련 가중치
    w_dynamics_  = this->declare_parameter("w_dynamics", 1.0);

    // MUX에서 강제할 공통 속도/가속도 제약
    max_speed_mux_   = this->declare_parameter("max_speed_mux", 6.0);   // [m/s]
    max_dv_step_mux_ = this->declare_parameter("max_dv_step_mux", 1.0); // 인접 포인트 간 허용 속도 변화

    // ------- 구독 설정 -------
    sub_frenet_ = this->create_subscription<nav_msgs::msg::Path>(
      "/frenet_local_plan", 10, std::bind(&LocalPlannerMux::frenet_callback, this, _1));

    sub_fgm_ = this->create_subscription<nav_msgs::msg::Path>(
      "/fgm_path", 10, std::bind(&LocalPlannerMux::fgm_callback, this, _1));

    sub_global_ = this->create_subscription<nav_msgs::msg::Path>(
      "/plan", 10, std::bind(&LocalPlannerMux::global_callback, this, _1));

    sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&LocalPlannerMux::scan_callback, this, _1));

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/ego_racecar/odom", 10, std::bind(&LocalPlannerMux::odom_callback, this, _1));

    // ------- 발행 설정 -------
    pub_selected_ = this->create_publisher<nav_msgs::msg::Path>(
      "/selected_path", 10);

    // 20Hz 주기 타이머
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&LocalPlannerMux::control_loop, this));

    RCLCPP_INFO(this->get_logger(),
      "LocalPlannerMux started. d_min=%.2f, v_ref=%.2f, max_speed_mux=%.2f, a_lat_max=%.2f, a_brake_max=%.2f",
      d_min_, v_ref_, max_speed_mux_, a_lat_max_, a_brake_max_);
  }

private:
  // ===== 콜백들 =====
  void frenet_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    last_frenet_path_ = *msg;
    has_frenet_ = true;
  }

  void fgm_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    last_fgm_path_ = *msg;
    has_fgm_ = true;
  }

  void global_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    global_path_ = *msg;
    has_global_ = true;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    last_scan_ = *msg;
    has_scan_ = true;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odom_ = *msg;
    has_odom_ = true;
  }

  void log_metrics(const std::string & tag,
                   const Metrics & m,
                   double score,
                   bool safe)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "[MUX] %s: safe=%s, score=%.3f | "
      "min_obs_dist=%.3f, avg_v=%.3f, jerk=%.3f, track_err=%.3f, risk=%.3f, "
      "max_ay=%.3f, dyn_margin=%.3f",
      tag.c_str(),
      safe ? "true" : "false",
      score,
      m.min_obs_dist,
      m.avg_speed,
      m.jerk_cost,
      m.tracking_error,
      m.risk_index,
      m.max_lat_accel,
      m.dyn_clear_margin
    );
  }

  // ===== 메인 제어 루프 =====
  void control_loop()
  {
    // 필요한 입력들이 다 들어오기 전에는 아무 것도 하지 않음
    if (!has_frenet_ || !has_fgm_ || !has_global_ || !has_scan_ || !has_odom_) {
      return;
    }

    // 1) 평가용 path: 로컬 플래너가 만든 원본 그대로 사용
    nav_msgs::msg::Path eval_frenet = last_frenet_path_;
    nav_msgs::msg::Path eval_fgm    = last_fgm_path_;

    // 2) 라이다 + 오돔으로 장애물 포인트 집합 생성
    std::vector<Point2D> obs_points = build_obstacle_points(last_scan_, last_odom_);

    // 3) 각 경로에 대해 메트릭 계산
    Metrics mf = evaluate_path(eval_frenet, obs_points);
    Metrics mg = evaluate_path(eval_fgm,    obs_points);

    bool frenet_safe = is_path_safe(mf);
    bool fgm_safe    = is_path_safe(mg);

    // 3-1) 둘 다 score 계산 (정상/비상 구분 관계 없이 로그용)
    double sf = score_normal(mf);
    double sg = score_normal(mg);

    // 3-2) 메트릭/score/safe 여부를 로그로 찍기
    log_metrics("FRENET", mf, sf, frenet_safe);
    log_metrics("FGM   ", mg, sg, fgm_safe);

    nav_msgs::msg::Path selected;
    bool publish_needed = false;
    const char * selected_name = "NONE";
    const char * mode_name     = "NONE";

    // 4) 선택 로직
    if (frenet_safe && fgm_safe) {
      // ===== 정상 모드: 둘 다 안전 =====
      mode_name = "NORMAL";
      // 효율/편안함/동역학까지 포함된 score로 선택
      if (sf >= sg) {
        selected      = eval_frenet;
        selected_name = "FRENET";
      } else {
        selected      = eval_fgm;
        selected_name = "FGM";
      }
      publish_needed = true;
    }
    else {
      // ===== 비상 모드: 둘 중 하나 이상이 unsafe =====
      mode_name = "EMERGENCY";

      if (frenet_safe && !fgm_safe) {
        selected      = eval_frenet;
        selected_name = "FRENET";
        publish_needed = true;
      }
      else if (!frenet_safe && fgm_safe) {
        selected      = eval_fgm;
        selected_name = "FGM";
        publish_needed = true;
      }
      else {
        // 둘 다 unsafe면 그래도 "동역학 여유"와 "기하학 여유"가 더 좋은 쪽 선택
        double score_clear_f = mf.min_obs_dist + mf.dyn_clear_margin;
        double score_clear_g = mg.min_obs_dist + mg.dyn_clear_margin;
        if (score_clear_f >= score_clear_g) {
          selected      = eval_frenet;
          selected_name = "FRENET";
        } else {
          selected      = eval_fgm;
          selected_name = "FGM";
        }
        publish_needed = true;
      }
    }

    // 5) 최종 선택 결과 한 줄 요약 로그
    if (publish_needed) {
      RCLCPP_INFO(
        this->get_logger(),
        "[MUX] mode=%s, selected=%s",
        mode_name,
        selected_name);

      // 최종 선택된 경로에만 공통 속도/가속도 제약 적용
      enforce_speed_profile(selected);
      selected.header.stamp = this->now();
      pub_selected_->publish(selected);
    }
  }

  // ===== 안전 판정: 기하학 + 동역학 기준 =====
  bool is_path_safe(const Metrics & m) const
  {
    // 1) 순수 기하학적 최소 여유거리
    bool geom_ok = (m.min_obs_dist >= d_min_);

    // 2) 동역학 안전거리 여유 (모든 포인트에서 d_eff - d_dyn(v) >= 0 이면 OK)
    bool dyn_ok  = (m.dyn_clear_margin >= 0.0);

    // 3) 최대 횡가속도 제한
    bool lat_ok  = (std::fabs(m.max_lat_accel) <= a_lat_max_);

    return geom_ok && dyn_ok && lat_ok;
  }

  // ===== 속도를 MUX가 직접 목표치까지 끌어올리는 Boost 모드 =====
  void enforce_speed_profile(nav_msgs::msg::Path & path)
  {
    if (path.poses.empty()) return;

    double prev_v = 0.0;
    for (size_t i = 0; i < path.poses.size(); i++)
    {
      double v = path.poses[i].pose.position.z;
      if (!std::isfinite(v)) v = 0.0;

      // 1) 상한 제한
      v = std::min(v, max_speed_mux_);

      // 2) 인접 포인트 간 속도 변화 제한
      if (i > 0) {
        double dv = v - prev_v;
        if (dv >  max_dv_step_mux_) v = prev_v + max_dv_step_mux_;
        if (dv < -max_dv_step_mux_) v = prev_v - max_dv_step_mux_;
      }

      path.poses[i].pose.position.z = v;
      prev_v = v;
    }
  }

  // ===== 경로 평가 함수들 =====
  Metrics evaluate_path(const nav_msgs::msg::Path & path,
                        const std::vector<Point2D> & obs_points)
  {
    Metrics m;
    m.avg_speed       = compute_avg_speed(path);
    m.jerk_cost       = compute_jerk_cost(path);
    m.tracking_error  = compute_tracking_error(path, global_path_);
    m.min_obs_dist    = compute_min_obs_dist(path, obs_points);
    m.risk_index      = compute_risk_index(path, obs_points);

    // 동역학 관련 메트릭 계산
    compute_dynamics_metrics(path, obs_points, m);

    return m;
  }

  double compute_avg_speed(const nav_msgs::msg::Path & path) const
  {
    if (path.poses.empty()) return 0.0;
    double sum = 0.0;
    for (const auto & ps : path.poses) {
      sum += ps.pose.position.z; // z에 속도 저장
    }
    return sum / static_cast<double>(path.poses.size());
  }

  double compute_jerk_cost(const nav_msgs::msg::Path & path) const
  {
    size_t n = path.poses.size();
    if (n < 3) return jerk_ref_; // 너무 짧으면 벌점 비슷하게

    std::vector<double> v(n);
    for (size_t i = 0; i < n; ++i) {
      v[i] = path.poses[i].pose.position.z;
    }

    // 간단하게 ds=1로 가정 (상대 비교용)
    std::vector<double> a(n - 1);
    for (size_t i = 0; i + 1 < n; ++i) {
      a[i] = v[i + 1] - v[i];
    }

    double jerk_sum = 0.0;
    for (size_t i = 0; i + 1 < a.size(); ++i) {
      double j = a[i + 1] - a[i];
      jerk_sum += j * j;
    }

    return jerk_sum;
  }

  double compute_tracking_error(const nav_msgs::msg::Path & local,
                                const nav_msgs::msg::Path & global) const
  {
    if (local.poses.empty() || global.poses.empty()) {
      return track_ref_;
    }

    double sum = 0.0;
    for (const auto & lp : local.poses) {
      double x = lp.pose.position.x;
      double y = lp.pose.position.y;
      double min_d2 = std::numeric_limits<double>::infinity();

      for (const auto & gp : global.poses) {
        double dx = gp.pose.position.x - x;
        double dy = gp.pose.position.y - y;
        double d2 = dx * dx + dy * dy;
        if (d2 < min_d2) {
          min_d2 = d2;
        }
      }
      sum += std::sqrt(min_d2);
    }
    return sum / static_cast<double>(local.poses.size());
  }

  std::vector<Point2D> build_obstacle_points(
      const sensor_msgs::msg::LaserScan & scan,
      const nav_msgs::msg::Odometry & odom) const
  {
    std::vector<Point2D> points;
    int n = static_cast<int>(scan.ranges.size());
    points.reserve(n);

    double x0 = odom.pose.pose.position.x;
    double y0 = odom.pose.pose.position.y;

    // yaw 추출
    const auto & q = odom.pose.pose.orientation;
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);

    double angle = scan.angle_min;
    for (int i = 0; i < n; ++i, angle += scan.angle_increment) {
      double r = scan.ranges[i];
      if (!std::isfinite(r)) continue;
      if (r < scan.range_min || r > scan.range_max) continue;

      // 라이다 기준 좌표
      double lx = r * std::cos(angle);
      double ly = r * std::sin(angle);

      // 차량(odom) 기준으로 회전 + 평행이동
      double gx = x0 + cos_yaw * lx - sin_yaw * ly;
      double gy = y0 + sin_yaw * lx + cos_yaw * ly;

      points.push_back({gx, gy});
    }

    return points;
  }

  // 한 점에서의 "실효 여유 거리" (센터에서 차량 반경 뺀 값)
  double compute_eff_clearance_point(double x, double y,
                                     const std::vector<Point2D> & obs_points) const
  {
    if (obs_points.empty()) {
      return std::numeric_limits<double>::infinity();
    }

    double min_d2 = std::numeric_limits<double>::infinity();
    for (const auto & ob : obs_points) {
      double dx = ob.x - x;
      double dy = ob.y - y;
      double d2 = dx * dx + dy * dy;
      if (d2 < min_d2) {
        min_d2 = d2;
      }
    }

    double center_dist = std::sqrt(min_d2);
    double eff_dist    = center_dist - vehicle_radius_;

    if (!std::isfinite(eff_dist)) {
      return std::numeric_limits<double>::infinity();
    }
    return std::max(0.0, eff_dist);
  }

  double compute_min_obs_dist(const nav_msgs::msg::Path & path,
                              const std::vector<Point2D> & obs_points) const
  {
    if (path.poses.empty() || obs_points.empty()) {
      // 장애물이 없다고 보면 아주 큰 값 리턴
      return std::numeric_limits<double>::infinity();
    }

    double min_eff = std::numeric_limits<double>::infinity();
    for (const auto & ps : path.poses) {
      double x = ps.pose.position.x;
      double y = ps.pose.position.y;
      double eff = compute_eff_clearance_point(x, y, obs_points);
      if (eff < min_eff) {
        min_eff = eff;
      }
    }
    return min_eff;
  }

  double compute_risk_index(const nav_msgs::msg::Path & path,
                            const std::vector<Point2D> & obs_points) const
  {
    // 장애물이 없으면 충돌 위험도 0
    if (path.poses.empty() || obs_points.empty()) {
      return 0.0;
    }

    double max_risk = 0.0;

    for (const auto & ps : path.poses) {
      double x = ps.pose.position.x;
      double y = ps.pose.position.y;
      double v = ps.pose.position.z;

      if (!std::isfinite(v)) {
        v = 0.0;
      }
      v = std::max(0.0, v);

      // 이 포인트에서 가장 가까운 장애물까지의 eff 거리
      double d_eff = compute_eff_clearance_point(x, y, obs_points);

      // 정규화된 속도 (기준 속도 대비)
      double v_norm = 0.0;
      if (v_ref_ > 1e-3) {
        v_norm = v / v_ref_;
      }
      v_norm = std::max(0.0, v_norm);

      // 여유거리 기준으로 위험도 가중 (가까울수록 exp 항이 커짐)
      double c_ref = (clearance_ref_ > 0.1) ? clearance_ref_ : 0.1;
      double c_term = std::exp(-d_eff / c_ref);

      double risk = v_norm * c_term;
      if (risk > max_risk) {
        max_risk = risk;
      }
    }

    return max_risk;
  }

  // 세 점으로부터 곡률(kappa) 계산
  double compute_curvature_3pt(double x1, double y1,
                               double x2, double y2,
                               double x3, double y3) const
  {
    // 변 길이
    double a = std::hypot(x2 - x1, y2 - y1);
    double b = std::hypot(x3 - x2, y3 - y2);
    double c = std::hypot(x3 - x1, y3 - y1);

    double eps = 1e-4;
    if (a < eps || b < eps || c < eps) {
      return 0.0;
    }

    // 삼각형 면적 = |(AB x AC)| / 2
    double area2 = std::abs((x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1)); // 2*area

    double denom = a * b * c;
    if (denom < eps) {
      return 0.0;
    }

    double kappa = area2 / denom; // 2*area / (a b c)
    return kappa;
  }

  // 동역학 관련 메트릭 계산:
  //  - max_lat_accel: v^2 * kappa 의 최대값
  //  - dyn_clear_margin: min( d_eff - d_dyn(v) )
  void compute_dynamics_metrics(const nav_msgs::msg::Path & path,
                                const std::vector<Point2D> & obs_points,
                                Metrics & m) const
  {
    if (path.poses.empty()) {
      m.max_lat_accel   = 0.0;
      m.dyn_clear_margin = std::numeric_limits<double>::infinity();
      return;
    }

    double max_ay = 0.0;
    double min_margin = std::numeric_limits<double>::infinity();

    size_t n = path.poses.size();
    if (n == 1 || obs_points.empty()) {
      // 장애물이 없으면 동역학 여유는 무한, 횡가속도는 0으로 간주
      m.max_lat_accel   = 0.0;
      m.dyn_clear_margin = std::numeric_limits<double>::infinity();
      return;
    }

    for (size_t i = 0; i < n; ++i) {
      const auto & ps = path.poses[i];
      double x = ps.pose.position.x;
      double y = ps.pose.position.y;
      double v = ps.pose.position.z;

      if (!std::isfinite(v)) {
        v = 0.0;
      }
      v = std::max(0.0, v);

      // 1) eff 여유 거리
      double d_eff = compute_eff_clearance_point(x, y, obs_points);

      // 2) 동역학 안전거리: v * t_react + v^2 / (2 a_brake)
      double d_dyn = 0.0;
      if (a_brake_max_ > 1e-3) {
        d_dyn = v * react_time_ + (v * v) / (2.0 * a_brake_max_);
      }
      double margin = d_eff - d_dyn;
      if (margin < min_margin) {
        min_margin = margin;
      }

      // 3) 곡률 기반 횡가속도
      double kappa = 0.0;
      if (i > 0 && i + 1 < n) {
        const auto & p_prev = path.poses[i - 1];
        const auto & p_next = path.poses[i + 1];
        double x1 = p_prev.pose.position.x;
        double y1 = p_prev.pose.position.y;
        double x2 = x;
        double y2 = y;
        double x3 = p_next.pose.position.x;
        double y3 = p_next.pose.position.y;

        kappa = compute_curvature_3pt(x1, y1, x2, y2, x3, y3);
      }

      double ay = v * v * kappa;
      if (ay > max_ay) {
        max_ay = ay;
      }
    }

    m.max_lat_accel   = max_ay;
    m.dyn_clear_margin = min_margin;
  }

  double score_normal(const Metrics & m) const
  {
    auto clamp01 = [](double x) {
      return std::max(0.0, std::min(1.0, x));
    };

    double v_norm = (v_ref_ > 1e-3)     ? clamp01(m.avg_speed      / v_ref_)     : 0.0;
    double j_norm = (jerk_ref_ > 1e-3)  ? clamp01(m.jerk_cost      / jerk_ref_)  : 1.0;
    double t_norm = (track_ref_ > 1e-3) ? clamp01(m.tracking_error / track_ref_) : 1.0;

    // 최소 안전거리(d_min_)보다 얼마나 더 멀리 떨어져 있는지에 따른 여유 거리 점수
    double c_norm = 0.0;
    if (std::isfinite(m.min_obs_dist) && clearance_ref_ > 1e-3) {
      double margin = m.min_obs_dist - d_min_;
      c_norm = clamp01(margin / clearance_ref_);
    }

    // risk_index는 클수록 위험하므로 정규화 후 반대로 사용
    double r_norm = 0.0;
    if (risk_ref_ > 1e-3) {
      r_norm = clamp01(m.risk_index / risk_ref_);
    }

    // 동역학 관련 정규화: 최대 횡가속도는 작을수록 좋음
    double a_norm = 0.0;
    if (a_lat_max_ > 1e-3) {
      a_norm = clamp01(m.max_lat_accel / a_lat_max_);
    }

    // dyn_clear_margin이 0 이상이면 괜찮고, 음수면 큰 패널티
    double dyn_norm = 0.0;
    if (m.dyn_clear_margin < 0.0) {
      // 여유가 음수일수록 페널티 ↑, 대략 -1 m까지를 기준으로
      dyn_norm = clamp01(-m.dyn_clear_margin / 1.0); // margin -1m -> dyn_norm ~1
    }

    double score = 0.0;
    // 속도는 클수록 좋음
    score += w_speed_     * v_norm;
    // 추종오차/jerk/위험도/횡가속도/동역학 여유는 작을수록 좋음 → (1 - norm)
    score += w_track_     * (1.0 - t_norm);
    score += w_comfort_   * (1.0 - j_norm);
    score += w_clearance_ * c_norm;         // 여유 거리
    score += w_clearance_ * (1.0 - r_norm); // 동적 위험도
    score += w_dynamics_  * (1.0 - a_norm); // 횡가속도
    score += w_dynamics_  * (1.0 - dyn_norm); // 동역학 안전거리 여유

    return score;
  }

  // ===== 멤버 변수 =====
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_frenet_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_fgm_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_global_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_selected_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Path last_frenet_path_;
  nav_msgs::msg::Path last_fgm_path_;
  nav_msgs::msg::Path global_path_;
  sensor_msgs::msg::LaserScan last_scan_;
  nav_msgs::msg::Odometry last_odom_;

  bool has_frenet_ = false;
  bool has_fgm_    = false;
  bool has_global_ = false;
  bool has_scan_   = false;
  bool has_odom_   = false;

  // 파라미터
  double d_min_;
  double v_ref_;
  double jerk_ref_;
  double track_ref_;

  double w_speed_;
  double w_track_;
  double w_comfort_;
  double vehicle_radius_;
  double clearance_ref_;
  double risk_ref_;
  double w_clearance_;

  // 동역학 파라미터
  double a_lat_max_;
  double a_brake_max_;
  double react_time_;
  double w_dynamics_;

  double max_speed_mux_;
  double max_dv_step_mux_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalPlannerMux>());
  rclcpp::shutdown();
  return 0;
}
