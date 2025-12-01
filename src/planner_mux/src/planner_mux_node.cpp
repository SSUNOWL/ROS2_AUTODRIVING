// 파일명: planner_mux_node.cpp
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
  double min_obs_dist;
  double avg_speed;
  double jerk_cost;
  double tracking_error;
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
    d_min_      = this->declare_parameter("d_min", 0.45);   // FGM bubble_radius와 맞춤
    // 정규화 기준 값
    v_ref_      = this->declare_parameter("v_ref", 5.0);    // 기준 속도
    jerk_ref_   = this->declare_parameter("jerk_ref", 5.0); // 허용 jerk 크기
    track_ref_  = this->declare_parameter("track_ref", 0.5);// 허용 추종 오차 [m]

    // 효율 모드 가중치
    w_speed_    = this->declare_parameter("w_speed",   1.0);
    w_track_    = this->declare_parameter("w_track",   1.0);
    w_comfort_  = this->declare_parameter("w_comfort", 1.0);

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
      "LocalPlannerMux started. d_min=%.2f, v_ref=%.2f",
      d_min_, v_ref_);
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
      // 필요한 데이터가 다 모일 때까지 대기
      return;
    }

    // 장애물 포인트를 월드 좌표계로 변환 (라이다 프레임 == 차량 바디 프레임 가정)
    std::vector<Point2D> obs_points = build_obstacle_points(last_scan_, last_odom_);

    Metrics mf = evaluate_path(last_frenet_path_, obs_points);
    Metrics mg = evaluate_path(last_fgm_path_, obs_points);

    bool frenet_safe = (mf.min_obs_dist >= d_min_);
    bool fgm_safe    = (mg.min_obs_dist >= d_min_);

    nav_msgs::msg::Path selected;
    bool publish_needed = false;

    if (frenet_safe && fgm_safe) {
      // ===== 정상 모드: 둘 다 최소 안전거리 이상 =====
      double sf = score_normal(mf);
      double sg = score_normal(mg);

      selected = (sf >= sg) ? last_frenet_path_ : last_fgm_path_;
      publish_needed = true;

      RCLCPP_DEBUG(this->get_logger(),
        "NORMAL mode: mf(dist=%.2f, v=%.2f, J=%.2f, e=%.2f) "
        "mg(dist=%.2f, v=%.2f, J=%.2f, e=%.2f) -> %s",
        mf.min_obs_dist, mf.avg_speed, mf.jerk_cost, mf.tracking_error,
        mg.min_obs_dist, mg.avg_speed, mg.jerk_cost, mg.tracking_error,
        (sf >= sg) ? "Frenet" : "FGM");
    }
    else {
      // ===== 비상 모드: 한쪽이라도 d_min 미만 =====
      if (frenet_safe && !fgm_safe) {
        selected = last_frenet_path_;
        publish_needed = true;
        RCLCPP_WARN(this->get_logger(),
          "EMERGENCY mode: FGM unsafe (%.2f < %.2f). Use Frenet.",
          mg.min_obs_dist, d_min_);
      }
      else if (!frenet_safe && fgm_safe) {
        selected = last_fgm_path_;
        publish_needed = true;
        RCLCPP_WARN(this->get_logger(),
          "EMERGENCY mode: Frenet unsafe (%.2f < %.2f). Use FGM.",
          mf.min_obs_dist, d_min_);
      }
      else {
        // 둘 다 d_min 미만이면 그래도 더 멀리 떨어진 쪽 선택
        if (mf.min_obs_dist >= mg.min_obs_dist) {
          selected = last_frenet_path_;
        } else {
          selected = last_fgm_path_;
        }
        publish_needed = true;
        RCLCPP_ERROR(this->get_logger(),
          "EMERGENCY mode: BOTH unsafe (F=%.2f, G=%.2f, d_min=%.2f). "
          "Choose path with larger clearance.",
          mf.min_obs_dist, mg.min_obs_dist, d_min_);
      }
    }

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
    if (n < 3) return jerk_ref_; // 너무 짧으면 벌점

    std::vector<double> v(n);
    for (size_t i = 0; i < n; ++i) {
      v[i] = path.poses[i].pose.position.z;
    }

    // 간단하게 ds=1로 가정 (상대 비교용)
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
      // 장애물이 없다고 보면 아주 큰 값 리턴
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

  double score_normal(const Metrics & m)
  {
    auto clamp01 = [](double x) {
      return std::max(0.0, std::min(1.0, x));
    };

    double v_norm = (v_ref_ > 1e-3)     ? clamp01(m.avg_speed      / v_ref_)    : 0.0;
    double j_norm = (jerk_ref_ > 1e-3)  ? clamp01(m.jerk_cost      / jerk_ref_) : 1.0;
    double t_norm = (track_ref_ > 1e-3) ? clamp01(m.tracking_error / track_ref_): 1.0;

    // 속도는 클수록, jerk/추종오차는 작을수록 좋음
    double score = 0.0;
    score += w_speed_   * v_norm;
    score += w_track_   * (1.0 - t_norm);
    score += w_comfort_ * (1.0 - j_norm);
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
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalPlannerMux>());
  rclcpp::shutdown();
  return 0;
}
