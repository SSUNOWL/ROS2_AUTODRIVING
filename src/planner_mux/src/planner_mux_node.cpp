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
  double min_obs_dist;   // 장애물까지 최소 거리
  double avg_speed;      // 평균 속도 (z)
  double jerk_cost;      // jerk 기반 비용
  double tracking_error; // 전역 경로와의 평균 거리
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
    // 최소 안전거리 [m]
    d_min_ = this->declare_parameter("d_min", 0.45);

    // RACE 모드 기준
    track_race_thresh_ = this->declare_parameter("track_race_thresh", 0.3); // [m]
    d_clear_race_      = this->declare_parameter("d_clear_race", 1.5);      // [m] 이 정도면 충분히 여유

    // 정규화 기준 값
    track_ref_ = this->declare_parameter("track_ref", 0.5); // tracking_error 기준
    jerk_ref_  = this->declare_parameter("jerk_ref", 5.0);  // jerk 비용 기준

    // 경계 상황에서 점수 비교 시 가중치
    alpha_clear_ = this->declare_parameter("alpha_clear", 1.0);  // 장애물 여유 중요도
    alpha_track_ = this->declare_parameter("alpha_track", 1.0);  // Frenet tracking 중요도
    alpha_jerk_  = this->declare_parameter("alpha_jerk", 0.2);   // jerk 중요도
    beta_track_  = this->declare_parameter("beta_track", 0.2);   // FGM tracking 중요도(훨씬 낮게)

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

    RCLCPP_INFO(
      this->get_logger(),
      "LocalPlannerMux started. d_min=%.2f, track_race_thresh=%.2f, d_clear_race=%.2f",
      d_min_, track_race_thresh_, d_clear_race_);
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

  // ===== 메인 루프 =====
  void control_loop()
  {
    if (!has_frenet_ || !has_fgm_ || !has_global_ || !has_scan_ || !has_odom_) {
      return;
    }

    // 장애물 포인트 집합 생성 (Laser + Odom 기반, map 좌표계로 근사)
    std::vector<Point2D> obs_points = build_obstacle_points(last_scan_, last_odom_);

    // 각 경로에 대한 메트릭 계산
    Metrics mf = evaluate_path(last_frenet_path_, obs_points);
    Metrics mg = evaluate_path(last_fgm_path_,    obs_points);

    bool frenet_safe = (mf.min_obs_dist >= d_min_);
    bool fgm_safe    = (mg.min_obs_dist >= d_min_);

    nav_msgs::msg::Path selected;
    bool publish_needed = false;

    // ===== 1) EMERGENCY 모드: 둘 다 위험 =====
    if (!frenet_safe && !fgm_safe) {
      selected = (mf.min_obs_dist >= mg.min_obs_dist)
               ? last_frenet_path_
               : last_fgm_path_;

      publish_needed = true;

      RCLCPP_ERROR(
        this->get_logger(),
        "[EMERGENCY] BOTH unsafe. "
        "Frenet min_dist=%.3f, FGM min_dist=%.3f, d_min=%.3f. "
        "Choose path with larger clearance: %s",
        mf.min_obs_dist, mg.min_obs_dist, d_min_,
        (mf.min_obs_dist >= mg.min_obs_dist) ? "FRENET" : "FGM");
    }

    // ===== 2) AVOID 모드: Frenet만 위험, FGM은 안전 =====
    else if (!frenet_safe && fgm_safe) {
      selected = last_fgm_path_;
      publish_needed = true;

      RCLCPP_WARN(
        this->get_logger(),
        "[AVOID] Frenet unsafe (%.3f < d_min=%.3f). Use FGM.",
        mf.min_obs_dist, d_min_);
    }

    // ===== 3) FGM만 위험, Frenet은 안전 =====
    else if (frenet_safe && !fgm_safe) {
      selected = last_frenet_path_;
      publish_needed = true;

      RCLCPP_WARN(
        this->get_logger(),
        "[RACE] FGM unsafe (%.3f < d_min=%.3f). Keep Frenet.",
        mg.min_obs_dist, d_min_);
    }

    // ===== 4) 둘 다 안전: RACE / 경계 상황 =====
    else {
      // RACE 모드: Frenet이 레이싱라인 잘 따라가고, 장애물도 충분히 멀다
      if (mf.tracking_error < track_race_thresh_ &&
          mf.min_obs_dist   > d_clear_race_) {

        selected = last_frenet_path_;
        publish_needed = true;

        RCLCPP_DEBUG(
          this->get_logger(),
          "[RACE] Both safe. Frenet closely follows global path "
          "and clearance is large (dist=%.3f, track_err=%.3f). Use Frenet.",
          mf.min_obs_dist, mf.tracking_error);
      } else {
        // 경계 상황: 둘 다 안전이긴 한데,
        // Frenet이 레이싱라인에서 살짝 벗어나기 시작했거나,
        // 장애물이 좀 가까운 편인 경우.
        double sf = score_frenet(mf);
        double sg = score_fgm(mg);

        selected = (sf >= sg) ? last_frenet_path_ : last_fgm_path_;
        publish_needed = true;

        RCLCPP_DEBUG(
          this->get_logger(),
          "[BOUNDARY] Both safe but near obstacles / off-line.\n"
          "  Frenet : dist=%.3f, e=%.3f, J=%.3f, score=%.3f\n"
          "  FGM    : dist=%.3f, e=%.3f, J=%.3f, score=%.3f\n"
          "  Selected: %s",
          mf.min_obs_dist, mf.tracking_error, mf.jerk_cost, sf,
          mg.min_obs_dist, mg.tracking_error, mg.jerk_cost, sg,
          (sf >= sg) ? "FRENET" : "FGM");
      }
    }

    // 최종 선택된 경로 publish
    if (publish_needed) {
      selected.header.stamp = this->now();
      pub_selected_->publish(selected);
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
    return m;
  }

  double compute_avg_speed(const nav_msgs::msg::Path & path)
  {
    if (path.poses.empty()) return 0.0;
    double sum = 0.0;
    for (const auto & ps : path.poses) {
      sum += ps.pose.position.z; // z에 속도 저장
    }
    return sum / static_cast<double>(path.poses.size());
  }

  double compute_jerk_cost(const nav_msgs::msg::Path & path)
  {
    size_t n = path.poses.size();
    if (n < 3) return jerk_ref_; // 너무 짧으면 기준값 정도로 반환

    std::vector<double> v(n);
    for (size_t i = 0; i < n; ++i) {
      v[i] = path.poses[i].pose.position.z;
    }

    std::vector<double> a(n - 1);
    for (size_t i = 0; i + 1 < n; ++i) {
      a[i] = v[i + 1] - v[i];
    }

    double J = 0.0;
    for (size_t i = 0; i + 1 < a.size(); ++i) {
      double j = a[i + 1] - a[i];
      J += j * j;
    }
    return J;
  }

  double compute_tracking_error(const nav_msgs::msg::Path & local,
                                const nav_msgs::msg::Path & global)
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
      const nav_msgs::msg::Odometry & odom)
  {
    std::vector<Point2D> pts;
    int n = static_cast<int>(scan.ranges.size());
    pts.reserve(n);

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
      float r = scan.ranges[i];
      if (!std::isfinite(r)) continue;
      if (r < scan.range_min || r > scan.range_max) continue;

      // 라이다 프레임에서의 점 (라이다 프레임 == 차량 바디 프레임이라고 가정)
      double lx = r * std::cos(angle);
      double ly = r * std::sin(angle);

      // 차량 위치 기준 월드 좌표
      double wx = x0 + cos_yaw * lx - sin_yaw * ly;
      double wy = y0 + sin_yaw * lx + cos_yaw * ly;

      pts.push_back({wx, wy});
    }
    return pts;
  }

  double compute_min_obs_dist(const nav_msgs::msg::Path & path,
                              const std::vector<Point2D> & obs_points)
  {
    if (path.poses.empty() || obs_points.empty()) {
      return std::numeric_limits<double>::infinity();
    }

    double min_d2 = std::numeric_limits<double>::infinity();
    for (const auto & ps : path.poses) {
      double x = ps.pose.position.x;
      double y = ps.pose.position.y;
      for (const auto & ob : obs_points) {
        double dx = ob.x - x;
        double dy = ob.y - y;
        double d2 = dx * dx + dy * dy;
        if (d2 < min_d2) {
          min_d2 = d2;
        }
      }
    }
    return std::sqrt(min_d2);
  }

  // ===== 정규화/점수 계산 =====
  double clamp01(double x) const
  {
    return std::max(0.0, std::min(1.0, x));
  }

  double norm_clear(double dist) const
  {
    // d_clear_race 이상이면 거의 1에 가깝게
    if (d_clear_race_ <= 1e-3) return 1.0;
    return clamp01(dist / d_clear_race_);
  }

  double norm_track(double e) const
  {
    if (track_ref_ <= 1e-3) return 1.0;
    return clamp01(e / track_ref_);  // 작을수록 좋음
  }

  double norm_jerk(double J) const
  {
    if (jerk_ref_ <= 1e-3) return 1.0;
    return clamp01(J / jerk_ref_);   // 작을수록 좋음
  }

  double score_frenet(const Metrics & m) const
  {
    double c  = norm_clear(m.min_obs_dist);       // 클수록 좋음
    double te = norm_track(m.tracking_error);     // 작을수록 좋음
    double j  = norm_jerk(m.jerk_cost);          // 작을수록 좋음

    // Frenet은 global path 추종이 중요
    double score = 0.0;
    score += alpha_clear_ * c;
    score += alpha_track_ * (1.0 - te);
    score += alpha_jerk_  * (1.0 - j);
    return score;
  }

  double score_fgm(const Metrics & m) const
  {
    double c  = norm_clear(m.min_obs_dist);
    double te = norm_track(m.tracking_error);
    double j  = norm_jerk(m.jerk_cost);

    // FGM은 clear/jerk가 더 중요하고, global tracking은 비중 낮게
    double score = 0.0;
    score += alpha_clear_ * c;
    score += beta_track_  * (1.0 - te);
    score += alpha_jerk_  * (1.0 - j);
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
  double track_race_thresh_;
  double d_clear_race_;

  double track_ref_;
  double jerk_ref_;

  double alpha_clear_;
  double alpha_track_;
  double alpha_jerk_;
  double beta_track_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalPlannerMux>());
  rclcpp::shutdown();
  return 0;
}
