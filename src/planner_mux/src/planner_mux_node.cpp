#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "planner_mux/msg/mux_status.hpp"


#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <limits>

using std::placeholders::_1;
using planner_mux::msg::MuxStatus;

struct Metrics {
  double min_obs_dist;
  double avg_speed;
  double jerk_cost;
  double tracking_error;
  double risk_index;

  double max_lat_accel;
  double dyn_clear_margin;
};

struct Point2D {
  double x;
  double y;
};

class LocalPlannerMux : public rclcpp::Node {
public:
  LocalPlannerMux() : Node("local_planner_mux") {
    d_min_      = this->declare_parameter("d_min", 0.15);
    v_ref_      = this->declare_parameter("v_ref", 5.0);
    jerk_ref_   = this->declare_parameter("jerk_ref", 5.0);
    track_ref_  = this->declare_parameter("track_ref", 0.5);

    w_speed_    = this->declare_parameter("w_speed",   1.0);
    w_track_    = this->declare_parameter("w_track",   1.0);
    w_comfort_  = this->declare_parameter("w_comfort", 1.0);

    vehicle_radius_ = this->declare_parameter("vehicle_radius", 0.15);
    clearance_ref_  = this->declare_parameter("clearance_ref", 0.5);
    risk_ref_       = this->declare_parameter("risk_ref", 1.0);
    w_clearance_    = this->declare_parameter("w_clearance", 1.0);

    a_lat_max_      = this->declare_parameter("a_lat_max", 7.0);
    a_brake_max_    = this->declare_parameter("a_brake_max", 8.0);
    react_time_     = this->declare_parameter("react_time", 0.3);
    w_dynamics_     = this->declare_parameter("w_dynamics", 1.0);

    max_speed_mux_   = this->declare_parameter("max_speed_mux", 6.0);
    max_dv_step_mux_ = this->declare_parameter("max_dv_step_mux", 0.25);

    // 히스테리시스 파라미터
    switch_min_interval_ = this->declare_parameter("switch_min_interval", 0.3); // 최소 0.3초 유지
    switch_score_margin_ = this->declare_parameter("switch_score_margin", 0.15); // 점수 0.15 이상 좋아야 갈아탐

    last_switch_time_ = this->now();
    
    sub_frenet_ = this->create_subscription<nav_msgs::msg::Path>(
      "/frenet_local_plan", 10,
      std::bind(&LocalPlannerMux::frenet_callback, this, _1));

    sub_fgm_ = this->create_subscription<nav_msgs::msg::Path>(
      "/fgm_path", 10,
      std::bind(&LocalPlannerMux::fgm_callback, this, _1));

    sub_global_ = this->create_subscription<nav_msgs::msg::Path>(
      "/plan", 10,
      std::bind(&LocalPlannerMux::global_callback, this, _1));

    sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&LocalPlannerMux::scan_callback, this, _1));

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/ego_racecar/odom", 10,
      std::bind(&LocalPlannerMux::odom_callback, this, _1));

    pub_selected_ = this->create_publisher<nav_msgs::msg::Path>(
      "/selected_path", 10);
    
    status_pub_ = this->create_publisher<MuxStatus>(
      "/mux_status", 10);


    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&LocalPlannerMux::control_loop, this));

    current_mode_ = "NONE";
    current_planner_ = "NONE";

    RCLCPP_INFO(this->get_logger(), "[MUX] started (quiet mode)");
  }

private:
  rclcpp::Publisher<MuxStatus>::SharedPtr status_pub_;

  void frenet_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    last_frenet_path_ = *msg;
    has_frenet_ = true;
  }

  void fgm_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    last_fgm_path_ = *msg;
    has_fgm_ = true;
  }

  void global_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    global_path_ = *msg;
    has_global_ = true;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    last_scan_ = *msg;
    has_scan_ = true;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    last_odom_ = *msg;
    has_odom_ = true;
  }

  void control_loop() {
    if (!has_frenet_ || !has_fgm_ || !has_global_ || !has_scan_ || !has_odom_)
      return;

    nav_msgs::msg::Path eval_frenet = last_frenet_path_;
    nav_msgs::msg::Path eval_fgm    = last_fgm_path_;

    std::vector<Point2D> obs = build_obstacle_points(last_scan_, last_odom_);

    Metrics mf = evaluate_path(eval_frenet, obs);
    Metrics mg = evaluate_path(eval_fgm,    obs);

    bool frenet_safe = is_path_safe(mf);
    bool fgm_safe    = is_path_safe(mg);

    double sf = score_normal(mf);
    double sg = score_normal(mg);

    // 1단계: "이론적으로" 무엇이 더 좋은지 먼저 결정
    std::string candidate_mode    = "NONE";
    std::string candidate_planner = "NONE";
    nav_msgs::msg::Path candidate_path;
    bool candidate_valid = false;

    if (frenet_safe && fgm_safe) {
      // 둘 다 안전할 때: NORMAL 모드, 점수 높은 쪽 선택
      candidate_mode = "NORMAL";
      if (sf >= sg) {
        candidate_planner = "FRENET";
        candidate_path = eval_frenet;
      } else {
        candidate_planner = "FGM";
        candidate_path = eval_fgm;
      }
      candidate_valid = true;
    } else {
      // 한쪽이라도 unsafe면 EMERGENCY 모드
      candidate_mode = "EMERGENCY";

      if (frenet_safe && !fgm_safe) {
        candidate_planner = "FRENET";
        candidate_path = eval_frenet;
        candidate_valid = true;
      }
      else if (!frenet_safe && fgm_safe) {
        candidate_planner = "FGM";
        candidate_path = eval_fgm;
        candidate_valid = true;
      }
      else {
        // ★ 둘 다 unsafe인 경우: min_d 절대 우선
        double df = mf.min_obs_dist;
        double dg = mg.min_obs_dist;

        const double eps = 1e-3;
        if (df > dg + eps) {
          candidate_planner = "FRENET";
          candidate_path = eval_frenet;
        } else if (dg > df + eps) {
          candidate_planner = "FGM";
          candidate_path = eval_fgm;
        } else {
          // min_d 거의 비슷하면 점수로 결정
          if (sf >= sg) {
            candidate_planner = "FRENET";
            candidate_path = eval_frenet;
          } else {
            candidate_planner = "FGM";
            candidate_path = eval_fgm;
          }
        }
        candidate_valid = true;
      }
    }

    if (!candidate_valid || candidate_planner == "NONE") {
      // 선택할 수 없는 상황이면 그냥 아무것도 안 보냄
      return;
    }

    // 2단계: 히스테리시스 적용 (최소 유지 시간 + 점수 차이 마진)
    auto now = this->now();
    bool allow_switch = true;

    if (current_planner_ != "NONE" &&
        candidate_planner != current_planner_) {

      // (1) 최소 유지 시간 체크
      double dt = (now - last_switch_time_).seconds();
      if (dt < switch_min_interval_) {
        allow_switch = false;
      } else {
        // (2) 점수 차이 충분히 나는지 체크
        double score_candidate =
          (candidate_planner == "FRENET") ? sf : sg;
        double score_curr =
          (current_planner_ == "FRENET") ? sf :
          (current_planner_ == "FGM")    ? sg : score_candidate;

        if (score_candidate < score_curr + switch_score_margin_) {
          allow_switch = false;
        }
      }
    }

    // 3단계: 스위치 불허면, 기존 플래너 유지
    std::string new_mode    = candidate_mode;
    std::string new_planner = candidate_planner;
    nav_msgs::msg::Path selected = candidate_path;

    if (!allow_switch && current_planner_ != "NONE") {
      new_planner = current_planner_;
      new_mode    = current_mode_;

      if (current_planner_ == "FRENET")
        selected = eval_frenet;
      else if (current_planner_ == "FGM")
        selected = eval_fgm;
    }

    // 4단계: 로그 + 스위치 시간 갱신
    bool mode_changed    = (new_mode    != current_mode_);
    bool planner_changed = (new_planner != current_planner_);

    if (mode_changed || planner_changed) {
      RCLCPP_INFO(this->get_logger(),
        "[MUX] mode=%s → %s | planner=%s → %s",
        current_mode_.c_str(), new_mode.c_str(),
        current_planner_.c_str(), new_planner.c_str());

      // 변경 시에만 두 궤적의 점수 + 구성 요소 로그 출력
      auto log_metrics = [this](const char* name,
                                const Metrics& m,
                                double score) {
        RCLCPP_INFO(
          this->get_logger(),
          "[MUX]  %s | score=%.3f | v=%.2f | jerk=%.3f | track=%.3f | "
          "min_d=%.3f | risk=%.3f | a_lat=%.3f | dyn_margin=%.3f",
          name,
          score,
          m.avg_speed,
          m.jerk_cost,
          m.tracking_error,
          m.min_obs_dist,
          m.risk_index,
          m.max_lat_accel,
          m.dyn_clear_margin
        );
      };

      log_metrics("FRENET", mf, sf);
      log_metrics("FGM",    mg, sg);

      if (planner_changed) {
        last_switch_time_ = now;
      }

      current_mode_    = new_mode;
      current_planner_ = new_planner;
    }

    // 5단계: 속도 프로파일 적용 후 publish
    enforce_speed_profile(selected);
    selected.header.stamp = this->now();
    pub_selected_->publish(selected);

    MuxStatus st;
    st.planner = new_planner;
    st.mode    = new_mode;
    st.min_d_frenet   = mf.min_obs_dist;
    st.min_d_fgm      = mg.min_obs_dist;
    st.track_frenet   = mf.tracking_error;
    st.track_fgm      = mg.tracking_error;
    st.selected_min_d = (new_planner == "FRENET" ? mf.min_obs_dist : mg.min_obs_dist);
    st.selected_track = (new_planner == "FRENET" ? mf.tracking_error : mg.tracking_error);

status_pub_->publish(st);

  }

  // ★ EMERGENCY 기준 현실화: dyn_clear_margin은 안전 판정에 사용하지 않음
  bool is_path_safe(const Metrics & m) const {
    bool geom_ok = (m.min_obs_dist >= d_min_);
    bool lat_ok  = (std::fabs(m.max_lat_accel) <= a_lat_max_);
    return geom_ok && lat_ok;
  }

  void enforce_speed_profile(nav_msgs::msg::Path & path) {
    if (path.poses.empty()) return;

    double prev_v = 0.0;
    for (size_t i = 0; i < path.poses.size(); i++) {
      double v = path.poses[i].pose.position.z;
      if (!std::isfinite(v)) v = 0.0;

      v = std::min(v, max_speed_mux_);

      if (i > 0) {
        double dv = v - prev_v;
        if (dv >  max_dv_step_mux_) v = prev_v + max_dv_step_mux_;
        if (dv < -max_dv_step_mux_) v = prev_v - max_dv_step_mux_;
      }
      path.poses[i].pose.position.z = v;
      prev_v = v;
    }
  }

  Metrics evaluate_path(const nav_msgs::msg::Path & path,
                        const std::vector<Point2D> & obs) {

    Metrics m;
    m.avg_speed       = compute_avg_speed(path);
    m.jerk_cost       = compute_jerk_cost(path);
    m.tracking_error  = compute_tracking_error(path, global_path_);
    m.min_obs_dist    = compute_min_obs_dist(path, obs);
    m.risk_index      = compute_risk_index(path, obs);

    compute_dynamics_metrics(path, obs, m);
    return m;
  }

  double compute_avg_speed(const nav_msgs::msg::Path & path) const {
    if (path.poses.empty()) return 0.0;
    double sum = 0.0;
    for (auto & p : path.poses) sum += p.pose.position.z;
    return sum / path.poses.size();
  }

  double compute_jerk_cost(const nav_msgs::msg::Path & path) const {
    if (path.poses.size() < 3) return jerk_ref_;
    std::vector<double> v(path.poses.size());
    for (size_t i = 0; i < v.size(); i++) v[i] = path.poses[i].pose.position.z;

    std::vector<double> a(v.size() - 1);
    for (size_t i = 0; i + 1 < v.size(); i++) a[i] = v[i+1] - v[i];

    double jerk_sum = 0.0;
    for (size_t i = 0; i + 1 < a.size(); i++)
      jerk_sum += std::pow(a[i+1] - a[i], 2);

    return jerk_sum;
  }

  double compute_tracking_error(const nav_msgs::msg::Path & local,
                                const nav_msgs::msg::Path & global) const {

    if (local.poses.empty() || global.poses.empty())
      return track_ref_;

    double sum = 0.0;
    for (auto & lp : local.poses) {
      double lx = lp.pose.position.x;
      double ly = lp.pose.position.y;

      double best = 1e9;
      for (auto & gp : global.poses) {
        double dx = gp.pose.position.x - lx;
        double dy = gp.pose.position.y - ly;
        best = std::min(best, dx*dx + dy*dy);
      }
      sum += std::sqrt(best);
    }
    return sum / local.poses.size();
  }

  std::vector<Point2D> build_obstacle_points(
      const sensor_msgs::msg::LaserScan & scan,
      const nav_msgs::msg::Odometry & odom) const {

    std::vector<Point2D> pts;
    double x0 = odom.pose.pose.position.x;
    double y0 = odom.pose.pose.position.y;

    const auto & q = odom.pose.pose.orientation;
    double siny = 2*(q.w*q.z + q.x*q.y);
    double cosy = 1 - 2*(q.y*q.y + q.z*q.z);
    double yaw  = std::atan2(siny, cosy);

    double c = std::cos(yaw);
    double s = std::sin(yaw);

    double angle = scan.angle_min;
    for (float r : scan.ranges) {
      if (std::isfinite(r) &&
          r >= scan.range_min && r <= scan.range_max) {

        double lx = r * std::cos(angle);
        double ly = r * std::sin(angle);

        double gx = x0 + c * lx - s * ly;
        double gy = y0 + s * lx + c * ly;
        pts.push_back({gx, gy});
      }
      angle += scan.angle_increment;
    }
    return pts;
  }

  double compute_eff_clear(const Point2D & p,
                           const std::vector<Point2D> & obs) const {

    double best = 1e9;
    for (auto & o : obs) {
      double dx = o.x - p.x;
      double dy = o.y - p.y;
      best = std::min(best, dx*dx + dy*dy);
    }
    double d = std::sqrt(best) - vehicle_radius_;
    return std::max(0.0, d);
  }

  double compute_min_obs_dist(const nav_msgs::msg::Path & path,
                              const std::vector<Point2D> & obs) const {
    if (path.poses.empty() || obs.empty())
      return 1e9;

    double best = 1e9;
    for (auto & ps : path.poses) {
      Point2D p{ps.pose.position.x, ps.pose.position.y};
      best = std::min(best, compute_eff_clear(p, obs));
    }
    return best;
  }

  double compute_risk_index(const nav_msgs::msg::Path & path,
                            const std::vector<Point2D> & obs) const {
    if (path.poses.empty() || obs.empty())
      return 0.0;

    double worst = 0.0;
    for (auto & ps : path.poses) {
      Point2D p{ps.pose.position.x, ps.pose.position.y};
      double v = std::max(0.0, ps.pose.position.z);
      double d = compute_eff_clear(p, obs);

      double v_norm = (v_ref_ > 0.1 ? v / v_ref_ : 0.0);
      double c_term = std::exp(-d / std::max(0.1, clearance_ref_));
      worst = std::max(worst, v_norm * c_term);
    }
    return worst;
  }

  double compute_curv(double x1,double y1,double x2,double y2,double x3,double y3) const {
    double a = std::hypot(x2-x1, y2-y1);
    double b = std::hypot(x3-x2, y3-y2);
    double c = std::hypot(x3-x1, y3-y1);
    if (a<1e-4 || b<1e-4 || c<1e-4) return 0.0;

    double area2 = std::abs((x2-x1)*(y3-y1) - (y2-y1)*(x3-x1));
    double denom = a*b*c;
    if (denom < 1e-4) return 0.0;
    return area2 / denom;
  }

  void compute_dynamics_metrics(const nav_msgs::msg::Path & path,
                                const std::vector<Point2D> & obs,
                                Metrics & m) const {

    if (path.poses.empty()) {
      m.max_lat_accel = 0.0;
      m.dyn_clear_margin = 1e9;
      return;
    }

    double max_ay = 0.0;
    double min_margin = 1e9;

    for (size_t i = 0; i < path.poses.size(); i++) {
      auto & ps = path.poses[i];
      Point2D p{ps.pose.position.x, ps.pose.position.y};
      double v = std::max(0.0, ps.pose.position.z);

      double d_eff = compute_eff_clear(p, obs);

      double d_dyn = v * react_time_ +
                     (a_brake_max_ > 1e-3 ? (v*v) / (2*a_brake_max_) : 0.0);

      min_margin = std::min(min_margin, d_eff - d_dyn);

      double kappa = 0.0;
      if (i>0 && i+1<path.poses.size()) {
        auto & p0 = path.poses[i-1];
        auto & p1 = path.poses[i];
        auto & p2 = path.poses[i+1];
        kappa = compute_curv(
          p0.pose.position.x, p0.pose.position.y,
          p1.pose.position.x, p1.pose.position.y,
          p2.pose.position.x, p2.pose.position.y
        );
      }

      max_ay = std::max(max_ay, v*v*kappa);
    }

    m.max_lat_accel = max_ay;
    m.dyn_clear_margin = min_margin;
  }

  double score_normal(const Metrics & m) const {
    auto c01 = [](double x){ return std::max(0.0, std::min(1.0, x)); };

    double v_norm = c01(m.avg_speed / std::max(0.1, v_ref_));
    double j_norm = c01(m.jerk_cost / std::max(0.1, jerk_ref_));
    double t_norm = c01(m.tracking_error / std::max(0.1, track_ref_));

    double c_norm = c01((m.min_obs_dist - d_min_) /
                        std::max(0.1, clearance_ref_));

    double r_norm = c01(m.risk_index / std::max(0.1, risk_ref_));

    double a_norm = c01(m.max_lat_accel / std::max(0.1, a_lat_max_));
    double dy_norm = (m.dyn_clear_margin < 0 ?
                      c01(-m.dyn_clear_margin / 1.0) : 0.0);

    double score = 0.0;
    score += w_speed_     * v_norm;
    score += w_track_     * (1.0 - t_norm);
    score += w_comfort_   * (1.0 - j_norm);
    score += w_clearance_ * c_norm;
    score += w_clearance_ * (1.0 - r_norm);
    score += w_dynamics_  * (1.0 - a_norm);
    score += w_dynamics_  * (1.0 - dy_norm);

    return score;
  }

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

  std::string current_mode_;
  std::string current_planner_;

  double d_min_, v_ref_, jerk_ref_, track_ref_;
  double w_speed_, w_track_, w_comfort_;
  double vehicle_radius_, clearance_ref_, risk_ref_, w_clearance_;
  double a_lat_max_, a_brake_max_, react_time_, w_dynamics_;
  double max_speed_mux_, max_dv_step_mux_;

  double switch_min_interval_;
  double switch_score_margin_;
  rclcpp::Time last_switch_time_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalPlannerMux>());
  rclcpp::shutdown();
  return 0;
}
