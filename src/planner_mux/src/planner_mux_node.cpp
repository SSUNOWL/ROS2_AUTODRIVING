#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "planner_mux_msgs/msg/mux_debug.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using planner_mux_msgs::msg::MuxDebug;

class PlannerMuxNode : public rclcpp::Node
{
public:
  PlannerMuxNode() : Node("planner_mux_node")
  {
    // Declare parameters
    w_speed_   = declare_parameter<double>("w_speed",   3.0);
    w_track_   = declare_parameter<double>("w_track",   2.0);
    w_comfort_ = declare_parameter<double>("w_comfort", 1.5);
    w_safety_  = declare_parameter<double>("w_safety",  3.0);

    v_ref_     = declare_parameter<double>("v_ref",     5.0);
    track_ref_ = declare_parameter<double>("track_ref", 0.5);
    jerk_ref_  = declare_parameter<double>("jerk_ref",  5.0);
    d_safe_    = declare_parameter<double>("d_safe",    0.45);

    frenet_path_sub_ =
      create_subscription<nav_msgs::msg::Path>(
        "/frenet_local_plan", 10,
        std::bind(&PlannerMuxNode::frenet_callback, this, std::placeholders::_1));

    fgm_path_sub_ =
      create_subscription<nav_msgs::msg::Path>(
        "/fgm_path", 10,
        std::bind(&PlannerMuxNode::fgm_callback, this, std::placeholders::_1));

    mux_pub_ =
      create_publisher<nav_msgs::msg::Path>("/selected_path", 10);

    debug_pub_ =
      create_publisher<MuxDebug>("/mux_debug", 10);
  }

private:

  // -----------------------
  // Frenet / FGM Callbacks
  // -----------------------
  void frenet_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    last_frenet_path_ = *msg;
    compute_and_publish_mux();
  }

  void fgm_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    last_fgm_path_ = *msg;
    compute_and_publish_mux();
  }

  // -----------------------
  // Score calculation utils
  // -----------------------
  double speed_term(double v)
  {
    return std::pow((v_ref_ - v) / v_ref_, 2);
  }

  double track_term(double track_err)
  {
    return std::pow(track_err / track_ref_, 2);
  }

  double comfort_term(double jerk, double a_lat)
  {
    return std::pow(jerk / jerk_ref_, 2)
         + std::pow(a_lat / 5.0, 2);
  }

  double safety_term(double min_d, double risk)
  {
    double d_term = std::pow(std::max(0.0, (d_safe_ - min_d) / d_safe_), 2);
    return d_term + risk * risk;
  }

  // -----------------------
  // Core MUX function
  // -----------------------
  void compute_and_publish_mux()
  {
    if (last_frenet_path_.poses.empty() || last_fgm_path_.poses.empty())
      return;

    // Dummy values (replace with real metrics)
    double v_fre = 4.0, v_fgm = 3.5;
    double track_fre = 0.2, track_fgm = 0.35;
    double jerk_fre = 1.0, jerk_fgm = 0.8;
    double a_lat_fre = 3.5, a_lat_fgm = 2.1;
    double min_d_fre = 0.6, min_d_fgm = 0.4;
    double risk_fre = 0.1, risk_fgm = 0.3;

    double fre_score =
        w_speed_   * speed_term(v_fre)
      + w_track_   * track_term(track_fre)
      + w_comfort_ * comfort_term(jerk_fre, a_lat_fre)
      + w_safety_  * safety_term(min_d_fre, risk_fre);

    double fgm_score =
        w_speed_   * speed_term(v_fgm)
      + w_track_   * track_term(track_fgm)
      + w_comfort_ * comfort_term(jerk_fgm, a_lat_fgm)
      + w_safety_  * safety_term(min_d_fgm, risk_fgm);

    bool use_frenet = fre_score <= fgm_score;

    // Publish MUX path
    nav_msgs::msg::Path out = use_frenet ?
      last_frenet_path_ : last_fgm_path_;
    mux_pub_->publish(out);

    // Publish Debug
    auto dbg = MuxDebug();
    dbg.header.stamp = now();
    dbg.current_planner = use_frenet ? "FRENET" : "FGM";

    dbg.frenet_score = fre_score;
    dbg.fgm_score = fgm_score;

    dbg.fre_min_d = min_d_fre;
    dbg.fre_track = track_fre;
    dbg.fre_jerk = jerk_fre;
    dbg.fre_speed = v_fre;
    dbg.fre_risk = risk_fre;
    dbg.fre_a_lat = a_lat_fre;
    dbg.fre_dyn_margin = 0.0;

    dbg.fgm_min_d = min_d_fgm;
    dbg.fgm_track = track_fgm;
    dbg.fgm_jerk = jerk_fgm;
    dbg.fgm_speed = v_fgm;
    dbg.fgm_risk = risk_fgm;
    dbg.fgm_a_lat = a_lat_fgm;
    dbg.fgm_dyn_margin = 0.0;

    debug_pub_->publish(dbg);
  }

  // Vars
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr frenet_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr fgm_path_sub_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mux_pub_;
  rclcpp::Publisher<MuxDebug>::SharedPtr debug_pub_;

  nav_msgs::msg::Path last_frenet_path_;
  nav_msgs::msg::Path last_fgm_path_;

  // Tunable weights
  double w_speed_, w_track_, w_comfort_, w_safety_;

  double v_ref_, track_ref_, jerk_ref_, d_safe_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerMuxNode>());
  rclcpp::shutdown();
  return 0;
}
