#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <filesystem>
#include <algorithm>
#include <tf2/LinearMath/Quaternion.h> 

using namespace std;

struct Point {
    double x, y;
};

struct MapInfo {
    double resolution;
    double origin_x;
    double origin_y;
    int width;
    int height;
    cv::Mat image;        
    cv::Mat dist_field;   
};

class OfflinePlanner : public rclcpp::Node {
public:
    OfflinePlanner() : Node("offline_planner") {
        this->declare_parameter("map_path", "");
        string map_yaml_path = this->get_parameter("map_path").as_string();

        if (map_yaml_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Please provide 'map_path' parameter.");
            return;
        }

        if (!load_map(map_yaml_path)) return;

        // 1. 진짜 맵 기반으로 중심선 추출 (Skeletonization)
        vector<Point> center_line = extract_centerline();
        if (center_line.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to extract centerline.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Centerline extracted with %zu points.", center_line.size());

        // 2. 경로 최적화 (Optimizer) - [수정됨: 벽 밀어내기 대폭 강화]
        vector<Point> optimized_path = optimize_path(center_line);

        // 3. 끊긴 길 잇기 (Interpolation)
        vector<Point> filled_path = interpolate_path(optimized_path, 0.2); 

        // 4. 다림질 (Smoothing) - [수정됨: 반복 횟수 축소 유지]
        vector<Point> final_path = smooth_final_path(filled_path);

        if (final_path.size() < 10) {
             RCLCPP_ERROR(this->get_logger(), "Path is too short!");
             return;
        }

        // 5. 속도 계산 - [수정됨: 코너 속도 보수적 설정 유지]
        vector<double> speeds = generate_velocity_profile(final_path);

        vector<double> yaws = calculate_yaw_profile(final_path);

        // 안전성 검사
        int fixed_points = 0;
        for (auto& p : final_path) {
            int px = (int)((p.x - map_info_.origin_x) / map_info_.resolution);
            int py = (int)((map_info_.height - (p.y - map_info_.origin_y) / map_info_.resolution));
            
            if (px >= 0 && px < map_info_.width && py >= 0 && py < map_info_.height) {
                float dist = map_info_.dist_field.at<float>(py, px);
                if (dist * map_info_.resolution < 0.3) { // 경고 기준 0.3m로 상향
                    fixed_points++;
                }
            }
        }
        if (fixed_points > 0) {
            RCLCPP_WARN(this->get_logger(), "Warning: %d points are within 0.3m of the wall!", fixed_points);
        }

        // 결과 저장
        save_to_csv(final_path, speeds, yaws, "raceline_with_yaw.csv");        
        
        // 6. Rviz 퍼블리시 설정
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/plan", 10);

        generate_path_msg(final_path, yaws);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000), 
            std::bind(&OfflinePlanner::timer_callback, this)); 
    }

private:
    MapInfo map_info_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;       
    nav_msgs::msg::Path stored_path_msg_;      

    void timer_callback() {
        if (stored_path_msg_.poses.empty()) return;
        stored_path_msg_.header.stamp = this->now();
        path_pub_->publish(stored_path_msg_);
    }

    // -------------------------------------------------------------
    // [1] Map Load & Skeletonization
    // -------------------------------------------------------------
    void thinning_iteration(cv::Mat& img, int iter) {
        cv::Mat marker = cv::Mat::zeros(img.size(), CV_8UC1);
        for (int i = 1; i < img.rows - 1; i++) {
            for (int j = 1; j < img.cols - 1; j++) {
                uchar p2 = img.at<uchar>(i-1, j);
                uchar p3 = img.at<uchar>(i-1, j+1);
                uchar p4 = img.at<uchar>(i, j+1);
                uchar p5 = img.at<uchar>(i+1, j+1);
                uchar p6 = img.at<uchar>(i+1, j);
                uchar p7 = img.at<uchar>(i+1, j-1);
                uchar p8 = img.at<uchar>(i, j-1);
                uchar p9 = img.at<uchar>(i-1, j-1);

                int A  = (p2 == 0 && p3 == 1) + (p3 == 0 && p4 == 1) +
                         (p4 == 0 && p5 == 1) + (p5 == 0 && p6 == 1) +
                         (p6 == 0 && p7 == 1) + (p7 == 0 && p8 == 1) +
                         (p8 == 0 && p9 == 1) + (p9 == 0 && p2 == 1);
                int B  = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
                int m1 = iter == 0 ? (p2 * p4 * p6) : (p2 * p4 * p8);
                int m2 = iter == 0 ? (p4 * p6 * p8) : (p2 * p6 * p8);

                if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0)
                    marker.at<uchar>(i, j) = 1;
            }
        }
        img &= ~marker;
    }

    void thinning(const cv::Mat& src, cv::Mat& dst) {
        dst = src.clone();
        dst /= 255;  // 0, 1로 변환
        cv::Mat prev = cv::Mat::zeros(dst.size(), CV_8UC1);
        cv::Mat diff;
        do {
            thinning_iteration(dst, 0);
            thinning_iteration(dst, 1);
            cv::absdiff(dst, prev, diff);
            dst.copyTo(prev);
        } while (cv::countNonZero(diff) > 0);
        dst *= 255;
    }

    bool load_map(const string& yaml_path) {
        try {
            YAML::Node config = YAML::LoadFile(yaml_path);
            map_info_.resolution = config["resolution"].as<double>();
            auto origin = config["origin"].as<std::vector<double>>();
            map_info_.origin_x = origin[0];
            map_info_.origin_y = origin[1];
            
            string image_name = config["image"].as<string>();
            namespace fs = std::filesystem;
            fs::path yaml_dir = fs::path(yaml_path).parent_path();
            fs::path image_path = yaml_dir / image_name;

            map_info_.image = cv::imread(image_path.string(), cv::IMREAD_GRAYSCALE);
            if (map_info_.image.empty()) return false;
            
            map_info_.width = map_info_.image.cols;
            map_info_.height = map_info_.image.rows;

            cv::Mat binary_map;
            cv::threshold(map_info_.image, binary_map, 200, 255, cv::THRESH_BINARY);
            
            cv::distanceTransform(binary_map, map_info_.dist_field, cv::DIST_L2, 5);

            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "YAML Error: %s", e.what());
            return false;
        }
    }

    vector<Point> extract_centerline() {
        RCLCPP_INFO(this->get_logger(), "Step 1: Processing Map...");
        cv::Mat binary_map;
        
        cv::threshold(map_info_.image, binary_map, 200, 255, cv::THRESH_BINARY);

        // FloodFill로 바깥 공간 제거
        if (binary_map.at<uchar>(0, 0) == 255) {
            cv::floodFill(binary_map, cv::Point(0, 0), cv::Scalar(0));
            RCLCPP_INFO(this->get_logger(), "Outer free space removed using FloodFill.");
        }
        
        if (binary_map.at<uchar>(0, binary_map.cols-1) == 255) 
            cv::floodFill(binary_map, cv::Point(binary_map.cols-1, 0), cv::Scalar(0));
        if (binary_map.at<uchar>(binary_map.rows-1, 0) == 255) 
            cv::floodFill(binary_map, cv::Point(0, binary_map.rows-1), cv::Scalar(0));
        if (binary_map.at<uchar>(binary_map.rows-1, binary_map.cols-1) == 255) 
            cv::floodFill(binary_map, cv::Point(binary_map.cols-1, binary_map.rows-1), cv::Scalar(0));

        RCLCPP_INFO(this->get_logger(), "Step 2: Thinning..."); 
        cv::Mat skeleton;
        thinning(binary_map, skeleton);
        
        vector<vector<cv::Point>> contours;
        cv::findContours(skeleton, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

        if (contours.empty()) return {};

        auto max_contour = std::max_element(contours.begin(), contours.end(),
            [](const vector<cv::Point>& a, const vector<cv::Point>& b) {
                return a.size() < b.size();
            });

        // 4. 픽셀 좌표 -> 월드 좌표 변환
        vector<Point> path;
        int step = 5; 
        for (size_t i = 0; i < max_contour->size(); i += step) {
            cv::Point p = (*max_contour)[i];
            
            Point wp;
            wp.x = p.x * map_info_.resolution + map_info_.origin_x;
            wp.y = (map_info_.height - p.y) * map_info_.resolution + map_info_.origin_y;
            path.push_back(wp);
        }
        
        if (path.empty()) return {};

        int best_start_idx = 0;
        double max_safety = -1.0;

        for (size_t i = 0; i < path.size(); i++) {
            int px = (int)((path[i].x - map_info_.origin_x) / map_info_.resolution);
            int py = (int)((map_info_.height - (path[i].y - map_info_.origin_y) / map_info_.resolution)); 

            float dist = 0.0f;
            if (px >= 0 && px < map_info_.width && py >= 0 && py < map_info_.height) {
                dist = map_info_.dist_field.at<float>(py, px);
                
                if (dist > max_safety && dist < 200.0) { 
                    max_safety = dist;
                    best_start_idx = i;
                }
            }
        }
        
        if (best_start_idx > 0) {
            std::rotate(path.begin(), path.begin() + best_start_idx, path.end());
            RCLCPP_INFO(this->get_logger(), "Path rotated!");
        }

        return path;
    }

    // [수정된 함수] optimize_path: 벽 밀어내기 힘 최대화
    vector<Point> optimize_path(vector<Point> path) {
        RCLCPP_INFO(this->get_logger(), "Step 3: Optimizing Path (Maximize Safety Margin)...");
        
        vector<Point> new_path = path;
        int n_points = path.size();
        
        // [변경] 안전 마진을 0.65m -> 0.85m로 대폭 확대 (차량 폭의 약 3배)
        double safe_margin_meter = 0.85; 
        double safe_dist_px = safe_margin_meter / map_info_.resolution;

        // [변경] Alpha(직선화) 최소화, Beta(회피) 극대화
        // Alpha를 낮춰서 경로가 뻣뻣하게 펴지며 벽을 파고드는 현상 방지
        double alpha = 0.02;  // 0.05 -> 0.02 
        double beta = 0.80;   // 0.65 -> 0.80 
        
        int iterations = 3000; 
        double learning_rate = 0.30; 

        int max_x = map_info_.width - 1;
        int max_y = map_info_.height - 1;

        for (int iter = 0; iter < iterations; iter++) {
            for (int i = 1; i < n_points - 1; i++) {
                Point& curr = new_path[i];
                Point prev = new_path[i-1];
                Point next = new_path[i+1];

                double smooth_dx = (prev.x + next.x) * 0.5 - curr.x;
                double smooth_dy = (prev.y + next.y) * 0.5 - curr.y;

                int px = (int)((curr.x - map_info_.origin_x) / map_info_.resolution);
                int py = (int)((map_info_.height - (curr.y - map_info_.origin_y) / map_info_.resolution)); 

                double obs_dx = 0.0;
                double obs_dy = 0.0;
                
                double current_beta = beta; // 동적 베타

                if (px >= 1 && px < max_x && py >= 1 && py < max_y) {
                    float dist = map_info_.dist_field.at<float>(py, px);

                    // [추가] 50cm 이내면 밀어내는 힘 4배 증폭 (Emergency Push)
                    // 기준 거리를 30cm -> 50cm로 늘려 더 일찍 반응하게 함
                    if (dist * map_info_.resolution < 0.5) {
                        current_beta *= 4.0; 
                    }

                    if (dist < safe_dist_px) {
                          float dx = map_info_.dist_field.at<float>(py, px+1) - map_info_.dist_field.at<float>(py, px-1);
                          float dy = map_info_.dist_field.at<float>(py+1, px) - map_info_.dist_field.at<float>(py-1, px);
                          
                          float len = std::sqrt(dx*dx + dy*dy);
                          if (len > 1e-3) {
                              obs_dx = dx / len;
                              obs_dy = -dy / len; 

                              // [추가] 가까울수록 가중치 대폭 추가 (비례 제어 강화)
                              // 1.0 + push_factor * 3.0 -> 가까우면 최대 4배 더 세게 밈
                              double push_factor = (safe_dist_px - dist) / safe_dist_px; 
                              obs_dx *= (1.0 + push_factor * 3.0);
                              obs_dy *= (1.0 + push_factor * 3.0);
                          }
                    }
                } else {
                    curr.x = (prev.x + next.x) * 0.5;
                    curr.y = (prev.y + next.y) * 0.5;
                    continue;
                }

                curr.x += (alpha * smooth_dx + current_beta * obs_dx) * learning_rate;
                curr.y += (alpha * smooth_dy + current_beta * obs_dy) * learning_rate;

                int new_px = (int)((curr.x - map_info_.origin_x) / map_info_.resolution);
                int new_py = (int)((map_info_.height - (curr.y - map_info_.origin_y) / map_info_.resolution));

                bool is_unsafe = false;
                
                if (new_px < 0 || new_px >= map_info_.width || new_py < 0 || new_py >= map_info_.height) {
                    is_unsafe = true;
                } else {
                    float new_dist = map_info_.dist_field.at<float>(new_py, new_px);
                    if (new_dist < 1.0) { 
                        is_unsafe = true;
                    }
                }

                if (is_unsafe) {
                    curr.x = (prev.x + next.x) * 0.5;
                    curr.y = (prev.y + next.y) * 0.5;
                }
            }
        }
        return new_path;
    }

    // [수정된 함수] smooth_final_path: 과도한 스무딩 방지
    vector<Point> smooth_final_path(const vector<Point>& path) {
        vector<Point> smoothed = path;
        int n = path.size();
        
        // [유지] 반복 횟수 10회 (최적화된 경로 유지)
        int iterations = 10; 
        
        for (int iter = 0; iter < iterations; iter++) {
            for (int i = 1; i < n - 1; i++) {
                // [유지] 본인 위치 가중치 0.6
                smoothed[i].x = 0.2 * smoothed[i-1].x + 0.6 * smoothed[i].x + 0.2 * smoothed[i+1].x;
                smoothed[i].y = 0.2 * smoothed[i-1].y + 0.6 * smoothed[i].y + 0.2 * smoothed[i+1].y;
            }
        }
        return smoothed;
    }

    vector<Point> interpolate_path(const vector<Point>& path, double max_gap_meters) {
        vector<Point> dense_path;
        if (path.empty()) return dense_path;
        dense_path.push_back(path[0]);

        for (size_t i = 1; i < path.size(); i++) {
            Point p1 = path[i-1];
            Point p2 = path[i];
            double dist = std::hypot(p2.x - p1.x, p2.y - p1.y);

            if (dist > max_gap_meters) {
                int num_fill = std::ceil(dist / 0.1); 
                for (int j = 1; j < num_fill; j++) {
                    double ratio = (double)j / num_fill;
                    Point p_new;
                    p_new.x = p1.x + (p2.x - p1.x) * ratio;
                    p_new.y = p1.y + (p2.y - p1.y) * ratio;
                    dense_path.push_back(p_new);
                }
            }
            dense_path.push_back(p2);
        }
        return dense_path;
    }

    void generate_path_msg(const vector<Point>& path, const vector<double>& yaws) {
        stored_path_msg_.poses.clear(); 
        stored_path_msg_.header.frame_id = "map";
        
        for (size_t i = 0; i < path.size(); i++) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = path[i].x;
            pose.pose.position.y = path[i].y;
            pose.pose.position.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0, 0, yaws[i]); 

            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.pose.orientation.w = q.w();
            
            stored_path_msg_.poses.push_back(pose);
        }
    }

    double get_curvature(Point p1, Point p2, Point p3) {
        double D = 2 * (p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y));
        if (std::abs(D) < 1e-5) return 0.0;
        double dist_12 = std::hypot(p1.x - p2.x, p1.y - p2.y);
        double dist_23 = std::hypot(p2.x - p3.x, p2.y - p3.y);
        double dist_31 = std::hypot(p3.x - p1.x, p3.y - p1.y);
        double R = (dist_12 * dist_23 * dist_31) / std::abs(D);
        return 1.0 / R; 
    }

    // [수정된 함수] generate_velocity_profile: 코너 속도 보수적 설정 유지
    std::vector<double> generate_velocity_profile(const std::vector<Point>& path) {
        RCLCPP_INFO(this->get_logger(), "Step 4: Generating Velocity Profile...");
        size_t n = path.size();
        std::vector<double> velocity(n, 0.0);

        // [유지] 마찰 계수(mu) 0.4
        double mu = 0.4;       
        double g = 9.81;       
        double max_v = 7.0;    
        double max_accel = 3.0; 
        double max_decel = 2.0; 

        for (size_t i = 1; i < n - 1; i++) {
            double k = get_curvature(path[i-1], path[i], path[i+1]);
            if (k < 1e-3) {
                velocity[i] = max_v;
            } else {
                double v_limit = std::sqrt((mu * g) / k);
                velocity[i] = std::min(max_v, v_limit);
            }
        }
        velocity[0] = velocity[n-1] = 2.0; 

        for (int i = n - 2; i >= 0; i--) {
            double dist = std::hypot(path[i+1].x - path[i].x, path[i+1].y - path[i].y);
            double max_reachable = std::sqrt(std::pow(velocity[i+1], 2) + 2 * max_decel * dist);
            velocity[i] = std::min(velocity[i], max_reachable);
        }

        for (size_t i = 1; i < n; i++) {
            double dist = std::hypot(path[i].x - path[i-1].x, path[i].y - path[i-1].y);
            double max_reachable = std::sqrt(std::pow(velocity[i-1], 2) + 2 * max_accel * dist);
            velocity[i] = std::min(velocity[i], max_reachable);
        }
        return velocity;
    }

    double get_dist_to_wall(Point p) {
        int px = (int)((p.x - map_info_.origin_x) / map_info_.resolution);
        int py = (int)(map_info_.height - (p.y - map_info_.origin_y) / map_info_.resolution);
        if (px >= 0 && px < map_info_.width && py >= 0 && py < map_info_.height) {
            return map_info_.dist_field.at<float>(py, px) * map_info_.resolution;
        }
        return 0.0;
    }

    void save_to_csv(const vector<Point>& path, const vector<double>& speeds, const vector<double>& yaws, string filename) {        ofstream file(filename);
        file << "# x, y, yaw_rad, speed_mps, dist_to_wall\n";
        for (size_t i = 0; i < path.size(); i++) {
            double dist = get_dist_to_wall(path[i]);
            file << path[i].x << "," << path[i].y << "," << yaws[i] << "," << speeds[i] << "," << dist << "\n";        
        }
        file.close();
        RCLCPP_INFO(this->get_logger(), "Saved raceline to %s", filename.c_str());
    }

    std::vector<double> calculate_yaw_profile(const std::vector<Point>& path) {
        std::vector<double> yaws;
        for (size_t i = 0; i < path.size(); ++i) {
            double dx, dy;
            if (i < path.size() - 1) {
                dx = path[i+1].x - path[i].x;
                dy = path[i+1].y - path[i].y;
            } else {
                dx = path[i].x - path[i-1].x;
                dy = path[i].y - path[i-1].y;
            }
            yaws.push_back(std::atan2(dy, dx));
        }
        return yaws;
    }
    
    // generate_path_msg 중복 정의 삭제됨 (이미 클래스 상단에 구현됨)
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OfflinePlanner>());
    rclcpp::shutdown();
    return 0;
}