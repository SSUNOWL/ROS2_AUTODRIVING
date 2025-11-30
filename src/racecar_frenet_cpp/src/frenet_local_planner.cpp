#include <memory>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>  // std::clamp

#include <Eigen/Dense>  // Eigen 사용

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using std::placeholders::_1;

struct Waypoint {
  double x;
  double y;
  double v;
};

struct RefPoint {
  double s;
  double x;
  double y;
};

struct FrenetPath
{
  std::vector<double> t;
  std::vector<double> d;
  std::vector<double> d_d;
  std::vector<double> d_dd;
  std::vector<double> s;
  std::vector<double> s_d;
  std::vector<double> s_dd;

  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> yaw;
  std::vector<double> v;
  std::vector<double> a;
  std::vector<double> c;

  double cf = 0.0;  // total cost
};

// quintic polynomial for lateral motion
class QuinticPolynomial
{
public:
  QuinticPolynomial(double xs, double vxs, double axs,
                    double xe, double vxe, double axe, double T)
  {
    a0_ = xs;
    a1_ = vxs;
    a2_ = axs / 2.0;

    Eigen::Matrix3d A;
    A << std::pow(T, 3), std::pow(T, 4), std::pow(T, 5),
         3 * std::pow(T, 2), 4 * std::pow(T, 3), 5 * std::pow(T, 4),
         6 * T, 12 * std::pow(T, 2), 20 * std::pow(T, 3);

    Eigen::Vector3d B;
    B << xe - a0_ - a1_ * T - a2_ * std::pow(T, 2),
         vxe - a1_ - 2 * a2_ * T,
         axe - 2 * a2_;

    Eigen::Vector3d x = A.colPivHouseholderQr().solve(B);
    a3_ = x(0);
    a4_ = x(1);
    a5_ = x(2);
  }

  double calc_point(double t) const
  {
    return a0_ + a1_ * t + a2_ * t * t
         + a3_ * std::pow(t, 3)
         + a4_ * std::pow(t, 4)
         + a5_ * std::pow(t, 5);
  }

  double calc_first_derivative(double t) const
  {
    return a1_
         + 2 * a2_ * t
         + 3 * a3_ * t * t
         + 4 * a4_ * std::pow(t, 3)
         + 5 * a5_ * std::pow(t, 4);
  }

  double calc_second_derivative(double t) const
  {
    return 2 * a2_
         + 6 * a3_ * t
         + 12 * a4_ * t * t
         + 20 * a5_ * std::pow(t, 3);
  }

private:
  double a0_, a1_, a2_, a3_, a4_, a5_;
};

// quartic polynomial for longitudinal motion
class QuarticPolynomial
{
public:
  QuarticPolynomial(double xs, double vxs, double axs,
                    double vxe, double axe, double T)
  {
    a0_ = xs;
    a1_ = vxs;
    a2_ = axs / 2.0;

    Eigen::Matrix3d A;
    A << 3 * std::pow(T, 2), 4 * std::pow(T, 3), 5 * std::pow(T, 4),
         6 * T, 12 * std::pow(T, 2), 20 * std::pow(T, 3),
         6, 24 * T, 60 * std::pow(T, 2);

    Eigen::Vector3d B;
    B << vxe - a1_ - 2 * a2_ * T,
         axe - 2 * a2_,
         0.0;

    Eigen::Vector3d x = A.colPivHouseholderQr().solve(B);
    a3_ = x(0);
    a4_ = x(1);
    a5_ = x(2);
  }

  double calc_point(double t) const
  {
    return a0_ + a1_ * t + a2_ * t * t
         + a3_ * std::pow(t, 3)
         + a4_ * std::pow(t, 4)
         + a5_ * std::pow(t, 5);
  }

  double calc_first_derivative(double t) const
  {
    return a1_
         + 2 * a2_ * t
         + 3 * a3_ * t * t
         + 4 * a4_ * std::pow(t, 3)
         + 5 * a5_ * std::pow(t, 4);
  }

  double calc_second_derivative(double t) const
  {
    return 2 * a2_
         + 6 * a3_ * t
         + 12 * a4_ * t * t
         + 20 * a5_ * std::pow(t, 3);
  }

private:
  double a0_, a1_, a2_, a3_, a4_, a5_;
};

class FrenetLocalPlanner : public rclcpp::Node
{
public:
  FrenetLocalPlanner()
  : Node("frenet_local_planner")
  {
    // ROS 파라미터
    max_speed_ = this->declare_parameter<double>("max_speed", 5.0);

    // Subscriber
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/ego_racecar/odom", 10,
      std::bind(&FrenetLocalPlanner::odomCallback, this, _1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&FrenetLocalPlanner::scanCallback, this, _1));

    global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/plan", 10,
      std::bind(&FrenetLocalPlanner::globalPathCallback, this, _1));

    // Publisher
    local_plan_pub_ =
      this->create_publisher<nav_msgs::msg::Path>("/frenet_local_plan", 10);

    // Timer
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&FrenetLocalPlanner::updateLocalPlan, this));

    RCLCPP_INFO(this->get_logger(), "Frenet Local Planner started.");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_odom_ = *msg;
    has_odom_ = true;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    latest_scan_ = *msg;
    has_scan_ = true;
  }

  void globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    global_path_ = *msg;
    preprocessGlobalPath();
    has_global_path_ = true;
    RCLCPP_INFO(this->get_logger(), "Received global path: %zu poses",
                global_path_.poses.size());
  }

  // ---------- 글로벌 Path 전처리 (s 축 추가) ----------
  void preprocessGlobalPath()
  {
    ref_points_.clear();
    if (global_path_.poses.size() < 2) return;

    double s_acc = 0.0;
    geometry_msgs::msg::PoseStamped prev = global_path_.poses.front();
    RefPoint rp0;
    rp0.s = s_acc;
    rp0.x = prev.pose.position.x;
    rp0.y = prev.pose.position.y;
    ref_points_.push_back(rp0);

    for (size_t i = 1; i < global_path_.poses.size(); ++i) {
      const auto & p = global_path_.poses[i];
      double dx = p.pose.position.x - prev.pose.position.x;
      double dy = p.pose.position.y - prev.pose.position.y;
      double ds = std::hypot(dx, dy);
      s_acc += ds;

      RefPoint rp;
      rp.s = s_acc;
      rp.x = p.pose.position.x;
      rp.y = p.pose.position.y;
      ref_points_.push_back(rp);

      prev = p;
    }

    RCLCPP_INFO(this->get_logger(),
                "Preprocessed global path to %zu ref points (total length = %.2f m)",
                ref_points_.size(), s_acc);
  }

  // ---------- (x,y) → (s,d) 변환 ----------
  bool xyToFrenet(double x, double y, double & s, double & d, size_t & nearest_idx)
  {
    if (ref_points_.size() < 2) return false;

    // 가장 가까운 ref point 찾기
    double min_dist = std::numeric_limits<double>::max();
    nearest_idx = 0;
    for (size_t i = 0; i < ref_points_.size(); ++i) {
      double dx = x - ref_points_[i].x;
      double dy = y - ref_points_[i].y;
      double dist = dx * dx + dy * dy;
      if (dist < min_dist) {
        min_dist = dist;
        nearest_idx = i;
      }
    }

    // s는 해당 ref point의 s
    s = ref_points_[nearest_idx].s;

    // d는 참조선 법선 방향으로의 거리
    if (nearest_idx == ref_points_.size() - 1) {
      d = 0.0;
      return true;
    }

    double x1 = ref_points_[nearest_idx].x;
    double y1 = ref_points_[nearest_idx].y;
    double x2 = ref_points_[nearest_idx + 1].x;
    double y2 = ref_points_[nearest_idx + 1].y;

    double vx = x2 - x1;
    double vy = y2 - y1;
    double v_norm = std::hypot(vx, vy);
    if (v_norm < 1e-6) {
      d = 0.0;
      return true;
    }
    vx /= v_norm;
    vy /= v_norm;

    // 참조선에서 현재점까지 벡터
    double wx = x - x1;
    double wy = y - y1;

    // lateral offset = cross(v, w)
    d = (-vy) * wx + vx * wy;

    return true;
  }

  void updateObstacles()
  {
    obstacles_.clear();

    if (!has_scan_ || !has_odom_) {
      return;
    }

    const auto & pose = latest_odom_.pose.pose;
    const double ex = pose.position.x;
    const double ey = pose.position.y;

    const auto & q = pose.orientation;
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    const double eyaw = std::atan2(siny_cosp, cosy_cosp);

    const auto & scan = latest_scan_;

    double angle = scan.angle_min;
    const int step = 1;

    for (size_t i = 0; i < scan.ranges.size(); i += step) {
      const double r = scan.ranges[i];

      if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) {
        angle += scan.angle_increment * step;
        continue;
      }

      const double x_l = r * std::cos(angle);
      const double y_l = r * std::sin(angle);

      angle += scan.angle_increment * step;

      const double x_m = ex + std::cos(eyaw) * x_l - std::sin(eyaw) * y_l;
      const double y_m = ey + std::sin(eyaw) * x_l + std::cos(eyaw) * y_l;

      obstacles_.emplace_back(x_m, y_m);
    }
  }

  // ---------- Frenet 최적 로컬 경로 생성 ----------
  bool generateLocalWaypoints(std::vector<Waypoint> & local_wps)
  {
    local_wps.clear();
    if (!has_odom_ || ref_points_.empty()) return false;

    // 장애물 관련 파라미터
    const double safe_radius    = 0.3;   // 이 안이면 충돌로 간주
    const double soft_radius    = 1.0;   // 이 안이면 cost 크게
    const double k_obs          = 5;  // 장애물 cost 가중치
    const double emergency_dist = 0.9;   // 이 안이면 감속/거의 정지

    // 1. 현재 상태 (x,y,yaw,v) → (s, d, s_d, d_d, d_dd)
    const auto & pose = latest_odom_.pose.pose;
    const auto & twist = latest_odom_.twist.twist;

    double x = pose.position.x;
    double y = pose.position.y;

    // yaw
    double qw = pose.orientation.w;
    double qx = pose.orientation.x;
    double qy = pose.orientation.y;
    double qz = pose.orientation.z;
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    double v = std::hypot(twist.linear.x, twist.linear.y);

    double s0, d0;
    size_t nearest_idx;
    if (!xyToFrenet(x, y, s0, d0, nearest_idx)) {
      return false;
    }
    double s_d0  = v;   // s 방향 속도 근사
    double s_dd0 = 0.0;
    double d_d0  = 0.0;
    double d_dd0 = 0.0;

    // 2. Frenet 후보 경로 생성
    std::vector<FrenetPath> fp_list;

    for (double di = -max_road_width_; di <= max_road_width_ + 1e-6; di += d_road_width_res_) {
      for (double Ti = mint_; Ti <= maxt_ + 1e-6; Ti += 1.0) {
        // lateral d(t): quintic
        QuinticPolynomial lat_qp(d0, d_d0, d_dd0,
                                 di, 0.0, 0.0, Ti);

        for (double tv = target_speed_ - d_t_s_ * n_s_sample_;
             tv <= target_speed_ + d_t_s_ * n_s_sample_ + 1e-6;
             tv += d_t_s_) {

          // longitudinal s(t): quartic
          QuarticPolynomial lon_qp(s0, s_d0, s_dd0,
                                   tv, 0.0, Ti);

          FrenetPath fp;
          // 시간 스텝별 샘플링
          for (double t = 0.0; t <= Ti + 1e-6; t += dt_) {
            fp.t.push_back(t);
            double di_t    = lat_qp.calc_point(t);
            double di_t_d  = lat_qp.calc_first_derivative(t);
            double di_t_dd = lat_qp.calc_second_derivative(t);

            double si_t    = lon_qp.calc_point(t);
            double si_t_d  = lon_qp.calc_first_derivative(t);
            double si_t_dd = lon_qp.calc_second_derivative(t);

            fp.d.push_back(di_t);
            fp.d_d.push_back(di_t_d);
            fp.d_dd.push_back(di_t_dd);

            fp.s.push_back(si_t);
            fp.s_d.push_back(si_t_d);
            fp.s_dd.push_back(si_t_dd);
          }

          fp_list.push_back(fp);
        }
      }
    }

    if (fp_list.empty()) return false;

    // 3. Frenet → Global 변환 + 제약/비용 계산
    for (auto & fp : fp_list) {
      bool collision = false;
      double min_dist = std::numeric_limits<double>::infinity();

      // Frenet (s, d) → (x, y)
      fp.x.resize(fp.s.size());
      fp.y.resize(fp.s.size());
      fp.yaw.resize(fp.s.size());
      fp.v.resize(fp.s.size());
      fp.a.resize(fp.s.size());
      fp.c.resize(fp.s.size());

      for (size_t i = 0; i < fp.s.size(); ++i) {
        double s = fp.s[i];
        double d = fp.d[i];

        // ref_points_에서 s에 가장 가까운 점 찾기
        size_t idx = 0;
        double min_ds = std::numeric_limits<double>::max();
        for (size_t j = 0; j < ref_points_.size(); ++j) {
          double ds = std::fabs(ref_points_[j].s - s);
          if (ds < min_ds) {
            min_ds = ds;
            idx = j;
          }
        }

        if (idx >= ref_points_.size() - 1) idx = ref_points_.size() - 2;

        double x1 = ref_points_[idx].x;
        double y1 = ref_points_[idx].y;
        double x2 = ref_points_[idx + 1].x;
        double y2 = ref_points_[idx + 1].y;

        double tx = x2 - x1;
        double ty = y2 - y1;
        double norm_t = std::hypot(tx, ty);
        if (norm_t < 1e-6) norm_t = 1e-6;
        tx /= norm_t;
        ty /= norm_t;
        // 노말 벡터
        double nx = -ty;
        double ny = tx;

        fp.x[i] = x1 + tx * (s - ref_points_[idx].s) + nx * d;
        fp.y[i] = y1 + ty * (s - ref_points_[idx].s) + ny * d;

        if (i > 0) {
          double dx_ = fp.x[i] - fp.x[i - 1];
          double dy_ = fp.y[i] - fp.y[i - 1];
          fp.yaw[i] = std::atan2(dy_, dx_);
          fp.v[i]   = fp.s_d[i];
          fp.a[i]   = fp.s_dd[i];
          if (i > 1) {
            double dyaw = fp.yaw[i] - fp.yaw[i - 1];
            dyaw = std::atan2(std::sin(dyaw), std::cos(dyaw));
            double ds = std::hypot(fp.x[i] - fp.x[i - 1],
                                   fp.y[i] - fp.y[i - 1]);
            fp.c[i] = ds < 1e-6 ? 0.0 : dyaw / ds;
          }
        } else {
          fp.yaw[i] = yaw;
          fp.v[i]   = fp.s_d[i];
          fp.a[i]   = fp.s_dd[i];
          fp.c[i]   = 0.0;
        }
      }

      // 장애물과의 최소거리 / 충돌 여부
      if (!obstacles_.empty()) {
        for (size_t i = 0; i < fp.x.size(); ++i) {
          const Eigen::Vector2d p(fp.x[i], fp.y[i]);
          for (const auto & obs : obstacles_) {
            const double dx_o = p.x() - obs.x();
            const double dy_o = p.y() - obs.y();
            const double dist = std::hypot(dx_o, dy_o);
            if (dist < min_dist) {
              min_dist = dist;
            }
            if (dist < safe_radius) {   // 너무 가까움 → 충돌 경로
              collision = true;
              break;
            }
          }
          if (collision) {
            break;
          }
        }
      }

      if (collision) {
        fp.cf = std::numeric_limits<double>::infinity();
        continue;
      }

      // 제약 위반 체크
      bool valid = true;
      for (size_t i = 0; i < fp.v.size(); ++i) {
        if (std::fabs(fp.v[i]) > max_speed_ + 1e-3) {
          valid = false; break;
        }
        if (std::fabs(fp.a[i]) > max_accel_ + 1e-3) {
          valid = false; break;
        }
        if (std::fabs(fp.c[i]) > max_curvature_ + 1e-3) {
          valid = false; break;
        }
      }
      if (!valid) {
        fp.cf = std::numeric_limits<double>::infinity();
        continue;
      }

      // 비용함수
      double J_lat = 0.0;
      double J_lon = 0.0;
      for (size_t i = 0; i < fp.d_dd.size(); ++i) {
        J_lat += fp.d_dd[i] * fp.d_dd[i];
      }
      for (size_t i = 0; i < fp.s_dd.size(); ++i) {
        J_lon += fp.s_dd[i] * fp.s_dd[i];
      }

      double cost_lat = J_lat;
      double cost_lon = J_lon
                      + (target_speed_ - fp.v.back())
                        * (target_speed_ - fp.v.back());
      double cost_d   = fp.d.back() * fp.d.back();  // 레인 중앙(0) 선호

      double cost_obs = 0.0;
      if (min_dist < soft_radius) {
        const double d_obs = std::max(min_dist, 0.1);
        cost_obs = k_obs * (1.0 / d_obs);
      }

      fp.cf = cost_lat + cost_lon + cost_d + cost_obs;
    }

    // 4. 최소 비용 경로 선택
    FrenetPath * best_fp = nullptr;
    double min_cost = std::numeric_limits<double>::infinity();
    for (auto & fp : fp_list) {
      if (fp.cf < min_cost) {
        min_cost = fp.cf;
        best_fp = &fp;
      }
    }

    if (best_fp == nullptr || !std::isfinite(min_cost)) {
      // 모든 경로가 충돌/제약 위반이면 → 그냥 제자리 정지 경로 생성
      local_wps.clear();
      Waypoint wp;
      wp.x = x;
      wp.y = y;
      wp.v = 0.0;
      local_wps.push_back(wp);
      return true;
    }

    // best path 기준 장애물 최소 거리 다시 계산
    double min_dist_best = std::numeric_limits<double>::infinity();
    if (!obstacles_.empty()) {
      for (size_t i = 0; i < best_fp->x.size(); ++i) {
        Eigen::Vector2d p(best_fp->x[i], best_fp->y[i]);
        for (const auto & obs : obstacles_) {
          double dx = p.x() - obs.x();
          double dy = p.y() - obs.y();
          double dist = std::hypot(dx, dy);
          if (dist < min_dist_best) {
            min_dist_best = dist;
          }
        }
      }
    }

    // 5. 선택된 경로를 Waypoint로 변환 (PP가 사용)
    local_wps.clear();
    for (size_t i = 0; i < best_fp->x.size(); ++i) {
      Waypoint wp;
      wp.x = best_fp->x[i];
      wp.y = best_fp->y[i];

      // 기본 속도
      double v_cmd = std::min(best_fp->v[i], max_speed_);

      // 장애물이 너무 가까우면 속도 줄이기
      if (min_dist_best < emergency_dist) {
        // min_dist_best == safe_radius 일 때 alpha = min_alpha
        // min_dist_best == emergency_dist 일 때 alpha = 1.0
        double alpha = (min_dist_best - safe_radius) / (emergency_dist - safe_radius);

        // 최소 속도 비율 (예: 0.4면 40%까지는 유지)
        const double min_alpha = 0.6;
        alpha = std::clamp(alpha, min_alpha, 1.0);

        v_cmd *= alpha;
        }

      wp.v = v_cmd;
      local_wps.push_back(wp);
    }

    return local_wps.size() >= 2;
  }

  // ---------- Path 퍼블리시 ----------
  void updateLocalPlan()
  {
    if (!has_odom_ || !has_global_path_) return;

    updateObstacles();

    std::vector<Waypoint> local_wps;
    if (!generateLocalWaypoints(local_wps)) return;

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "map";  // /plan 과 동일

    for (const auto & wp : local_wps) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path_msg.header;
      ps.pose.position.x = wp.x;
      ps.pose.position.y = wp.y;
      ps.pose.position.z = wp.v;   // z에 속도 저장 → PP에서 읽음
      ps.pose.orientation.w = 1.0;
      path_msg.poses.push_back(ps);
    }

    local_plan_pub_->publish(path_msg);
  }

  // ---------- 멤버 변수 ----------
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr      global_path_sub_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_plan_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Odometry      latest_odom_;
  sensor_msgs::msg::LaserScan  latest_scan_;
  nav_msgs::msg::Path          global_path_;
  std::vector<RefPoint>        ref_points_;
  std::vector<Eigen::Vector2d> obstacles_;

  bool has_odom_{false};
  bool has_scan_{false};
  bool has_global_path_{false};

  // 파라미터
  double max_speed_;              // [m/s] ROS 파라미터

  // Frenet 파라미터 (기본값)
  double max_accel_        = 2.0;   // [m/ss]
  double max_curvature_    = 1.0;   // [1/m]
  double max_road_width_   = 2.0;   // [m]  ← 회피 위해 늘림
  double d_road_width_res_ = 0.25;  // [m]  ← 더 촘촘하게
  double target_speed_     = 3.0;   // [m/s]
  double dt_               = 0.2;   // [s] time step
  double maxt_             = 3.0;   // [s] 최대 planning horizon
  double mint_             = 2.0;   // [s] 최소 planning horizon
  double d_t_s_            = 0.5;   // target speed 샘플링 간격
  double n_s_sample_       = 1.0;   // target speed 주변 샘플링 폭 배수
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrenetLocalPlanner>());
  rclcpp::shutdown();
  return 0;
}
