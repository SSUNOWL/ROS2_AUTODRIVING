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

<<<<<<< HEAD
        // 1. 진짜 맵 기반으로 중심선 추출 (Skeletonization)
        vector<Point> center_line = extract_centerline();
=======
        // 1. 진짜 맵 기반으로 중심선 추출
        vector<Point> center_line = extract_centerline();
        // [삭제됨] 디버그용 raw centerline 저장 코드 제거
        
>>>>>>> minimumlaps
        if (center_line.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to extract centerline.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Centerline extracted with %zu points.", center_line.size());

<<<<<<< HEAD
        // 2. 경로 최적화 (단순화 및 스무딩)
        vector<Point> optimized_path = optimize_path(center_line);


        // vector<Point> safe_path = prune_unsafe_points(optimized_path, 0.25);

        // if (safe_path.size() < 10) {
        //     RCLCPP_ERROR(this->get_logger(), "Path is too short after pruning! Check map or threshold.");
        //     return;
        // }
        // 기존 중심선 저장
        // save_to_csv(center_line, "path_1_centerline.csv"); 

        // 최적화된 경로 저장
        // save_to_csv(optimized_path, "path_2_optimized.csv");

            // [추가] 속도 계산
        vector<double> speeds = generate_velocity_profile(optimized_path);

        // 5. Rviz 시각화용 퍼블리시
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/optimal_path", 10);
        
        // 경로 메시지 생성 및 저장 (쏘지는 않음)
        generate_path_msg(optimized_path); 

        // [추가] 1초(1000ms)마다 timer_callback 실행 -> 경로 무한 발행
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000), 
            std::bind(&OfflinePlanner::timer_callback, this)); // [수정] save_to_csv 호출 시 speeds 전달
        save_to_csv(optimized_path, speeds, "raceline_with_speed.csv");
=======
        // 2. 경로 최적화 (Optimizer)
        vector<Point> optimized_path = optimize_path(center_line);

        // 3. 끊긴 길 잇기 (Interpolation)
        vector<Point> filled_path = interpolate_path(optimized_path, 0.2); 

        // 4. 다림질 (Smoothing)
        vector<Point> final_path = smooth_final_path(filled_path);

        if (final_path.size() < 10) {
             RCLCPP_ERROR(this->get_logger(), "Path is too short!");
             return;
        }

        // 5. 속도 계산
        vector<double> speeds = generate_velocity_profile(final_path);

        int fixed_points = 0;
        for (auto& p : final_path) {
            int px = (int)((p.x - map_info_.origin_x) / map_info_.resolution);
            int py = (int)((map_info_.height - (p.y - map_info_.origin_y) / map_info_.resolution));
            
            if (px >= 0 && px < map_info_.width && py >= 0 && py < map_info_.height) {
                float dist = map_info_.dist_field.at<float>(py, px);
                if (dist * map_info_.resolution < 0.2) { 
                    fixed_points++;
                }
            }
        }
        if (fixed_points > 0) {
            RCLCPP_WARN(this->get_logger(), "Warning: %d points are still potentially unsafe!", fixed_points);
        }

        // [유지] 최종 결과물 저장
        save_to_csv(final_path, speeds, "raceline_with_speed.csv");
        
        // 6. Rviz 퍼블리시 설정
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/optimal_path", 10);
        generate_path_msg(final_path); 

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000), 
            std::bind(&OfflinePlanner::timer_callback, this)); 
>>>>>>> minimumlaps
    }

private:
    MapInfo map_info_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
<<<<<<< HEAD

    rclcpp::TimerBase::SharedPtr timer_;       // 타이머
    nav_msgs::msg::Path stored_path_msg_;      // 경로 메시지 저장소

    void timer_callback() {
        if (stored_path_msg_.poses.empty()) return;
        
        // 타임스탬프를 현재 시간으로 갱신해서 보내야 RViz가 좋아합니다.
        stored_path_msg_.header.stamp = this->now();
        path_pub_->publish(stored_path_msg_);
    }
    // Zhang-Suen Thinning Algorithm (OpenCV 외부 모듈 의존성 제거용)
=======
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
>>>>>>> minimumlaps
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

<<<<<<< HEAD
            // 이미지 로드
=======
>>>>>>> minimumlaps
            map_info_.image = cv::imread(image_path.string(), cv::IMREAD_GRAYSCALE);
            if (map_info_.image.empty()) return false;
            
            map_info_.width = map_info_.image.cols;
            map_info_.height = map_info_.image.rows;

<<<<<<< HEAD
            // Binary Map 생성 (흰색: 주행 가능)
            cv::Mat binary_map;
            // Spielberg 맵이 흰색 배경에 검은 선이라면 반전이 필요할 수 있음
            // 보통 F1TENTH 맵은: 흰색(255)=주행가능, 검은색(0)=벽
            cv::threshold(map_info_.image, binary_map, 200, 255, cv::THRESH_BINARY);
            
            // Distance Map 생성 (벽과의 거리)
            cv::distanceTransform(binary_map, map_info_.dist_field, cv::DIST_L2, 5);

=======
            cv::Mat binary_map;
            cv::threshold(map_info_.image, binary_map, 200, 255, cv::THRESH_BINARY);
            
            cv::distanceTransform(binary_map, map_info_.dist_field, cv::DIST_L2, 5);

            // [삭제됨] 디버그용 이미지(debug_dist_field.png, debug_binary_map.png) 저장 코드 제거

>>>>>>> minimumlaps
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "YAML Error: %s", e.what());
            return false;
        }
    }

    vector<Point> extract_centerline() {
<<<<<<< HEAD
        RCLCPP_INFO(this->get_logger(), "Step 1: Thresholding..."); // 로그 추가
        cv::Mat binary_map;
        cv::threshold(map_info_.image, binary_map, 200, 255, cv::THRESH_BINARY_INV);
        
        // cv::imwrite("debug_1_binary.png", binary_map);

        RCLCPP_INFO(this->get_logger(), "Step 2: Start Thinning (This might be slow)..."); // 로그 추가
        cv::Mat skeleton;
        thinning(binary_map, skeleton);
        RCLCPP_INFO(this->get_logger(), "Step 2: Thinning Done."); // 로그 추가
        
        // cv::imwrite("debug_2_skeleton.png", skeleton);
        // 3. Contours 찾기 (점들을 순서대로 정렬하기 위함)
=======
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

        // [삭제됨] debug_floodfill_map.png 저장 코드 제거

        RCLCPP_INFO(this->get_logger(), "Step 2: Thinning..."); 
        cv::Mat skeleton;
        thinning(binary_map, skeleton);
        
>>>>>>> minimumlaps
        vector<vector<cv::Point>> contours;
        cv::findContours(skeleton, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

        if (contours.empty()) return {};

<<<<<<< HEAD
        // 가장 긴 컨투어(메인 트랙) 선택
=======
>>>>>>> minimumlaps
        auto max_contour = std::max_element(contours.begin(), contours.end(),
            [](const vector<cv::Point>& a, const vector<cv::Point>& b) {
                return a.size() < b.size();
            });

<<<<<<< HEAD
        // 4. 픽셀 좌표 -> 월드 좌표 변환
        vector<Point> path;
        // 너무 촘촘하면 계산이 느려지므로 적당히 샘플링 (Step 5)
        int step = 5; 
=======
        vector<Point> path;
        int step = 10; // 점 간격
>>>>>>> minimumlaps
        for (size_t i = 0; i < max_contour->size(); i += step) {
            cv::Point p = (*max_contour)[i];
            
            Point wp;
            wp.x = p.x * map_info_.resolution + map_info_.origin_x;
            wp.y = (map_info_.height - p.y) * map_info_.resolution + map_info_.origin_y;
<<<<<<< HEAD
            
            path.push_back(wp);
        }
=======
            path.push_back(wp);
        }
        
        if (path.empty()) return {};

        int best_start_idx = 0;
        double max_safety = -1.0;

        RCLCPP_INFO(this->get_logger(), "Checking path safety for %zu points...", path.size());

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

            if (i < 5 || i % 50 == 0) {
                RCLCPP_INFO(this->get_logger(), 
                    "Idx: %zu | Pixel(%d, %d) | Dist: %.4f", 
                    i, px, py, dist);
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Max Safety found: %.4f at Index %d", max_safety, best_start_idx);

        if (best_start_idx > 0) {
            std::rotate(path.begin(), path.begin() + best_start_idx, path.end());
            RCLCPP_INFO(this->get_logger(), "Path rotated!");
        } else {
            RCLCPP_INFO(this->get_logger(), "Start point is already optimal.");
        }

>>>>>>> minimumlaps
        return path;
    }

    vector<Point> optimize_path(vector<Point> path) {
<<<<<<< HEAD
        RCLCPP_INFO(this->get_logger(), "Step 3: Optimizing Path (Safe Margin applied)...");
=======
        RCLCPP_INFO(this->get_logger(), "Step 3: Optimizing Path (with Hard Constraints)...");
>>>>>>> minimumlaps
        
        vector<Point> new_path = path;
        int n_points = path.size();
        
<<<<<<< HEAD
        // --- [핵심 파라미터 튜닝] ---
        // 1. 차량 안전 거리 설정 (Car Width / 2 + Buffer)
        // F1TENTH 차폭이 약 0.3m이므로, 벽에서 최소 0.35m는 떨어져야 안전함
        double safe_margin_meter = 0.35; 
        double safe_dist_px = safe_margin_meter / map_info_.resolution; // 픽셀 단위로 변환

        // 2. 힘 조절 (Smoothness vs Repulsion)
        double alpha = 0.05;   // 선을 펴려는 힘 (너무 크면 코너 안쪽을 파고듦)
        double beta = 0.25;    // 벽에서 미는 힘 (충분히 커야 함)
        int iterations = 1000; // 반복 횟수
=======
        double safe_margin_meter = 0.40;
        double safe_dist_px = safe_margin_meter / map_info_.resolution;

        double alpha = 0.1;   
        double beta = 0.45;   
        int iterations = 3000; 
        double learning_rate = 0.4; 
>>>>>>> minimumlaps

        int max_x = map_info_.width - 1;
        int max_y = map_info_.height - 1;

        for (int iter = 0; iter < iterations; iter++) {
            for (int i = 1; i < n_points - 1; i++) {
                Point& curr = new_path[i];
                Point prev = new_path[i-1];
                Point next = new_path[i+1];

<<<<<<< HEAD
                // ------------------------------------
                // 1. Smoothness Term (직선화 힘)
                // ------------------------------------
                double smooth_dx = (prev.x + next.x) * 0.5 - curr.x;
                double smooth_dy = (prev.y + next.y) * 0.5 - curr.y;

                // ------------------------------------
                // 2. Obstacle Term (벽 회피 힘)
                // ------------------------------------
                // 월드 좌표 -> 픽셀 좌표 변환 (Y축 반전 여부 확인 필수!)
                // 아까 Y축 반전을 껐다면 아래 식을 사용:
                int px = (int)((curr.x - map_info_.origin_x) / map_info_.resolution);
                int py = (int)((curr.y - map_info_.origin_y) / map_info_.resolution); 
=======
                double smooth_dx = (prev.x + next.x) * 0.5 - curr.x;
                double smooth_dy = (prev.y + next.y) * 0.5 - curr.y;

                int px = (int)((curr.x - map_info_.origin_x) / map_info_.resolution);
                int py = (int)((map_info_.height - (curr.y - map_info_.origin_y) / map_info_.resolution)); 
>>>>>>> minimumlaps

                double obs_dx = 0.0;
                double obs_dy = 0.0;

<<<<<<< HEAD
                if (px > 1 && px < max_x - 1 && py > 1 && py < max_y - 1) {
                    // 현재 위치에서 벽까지의 거리 (픽셀 단위)
                    float dist = map_info_.dist_field.at<float>(py, px);
                    
                    // ★ 차량 크기(safe_dist_px)보다 가까우면 밀어냄
                    if (dist < safe_dist_px) {
                         // Distance Field의 기울기(Gradient) = 벽에서 멀어지는 방향
                         float dx = map_info_.dist_field.at<float>(py, px+1) - map_info_.dist_field.at<float>(py, px-1);
                         float dy = map_info_.dist_field.at<float>(py+1, px) - map_info_.dist_field.at<float>(py-1, px);
                         
                         // 벡터 정규화 (방향만 남김)
                         float len = std::sqrt(dx*dx + dy*dy);
                         if (len > 1e-3) {
                             // 거리가 가까울수록 더 세게 밀도록 가중치 추가 가능
                             // 여기서는 단순하게 방향만 적용
                             obs_dx = dx / len;
                             obs_dy = dy / len; 
                         }
                    }
                }

                // ------------------------------------
                // 3. 위치 업데이트
                // ------------------------------------
                curr.x += alpha * smooth_dx + beta * obs_dx;
                curr.y += alpha * smooth_dy + beta * obs_dy;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Optimization Complete. Safety Margin: %.2fm", safe_margin_meter);
        return new_path;
    }
    
    void generate_path_msg(const vector<Point>& path) {
        stored_path_msg_.header.frame_id = "map";
        // stored_path_msg_.header.stamp는 timer_callback에서 넣음

=======
                if (px < 1 || px >= max_x || py < 1 || py >= max_y) {
                    curr.x = (prev.x + next.x) / 0.5;
                    curr.y = (prev.y + next.y) / 0.5;
                    continue;
                }

                float dist = map_info_.dist_field.at<float>(py, px);

                if (dist < safe_dist_px) {
                      float dx = map_info_.dist_field.at<float>(py, px+1) - map_info_.dist_field.at<float>(py, px-1);
                      float dy = map_info_.dist_field.at<float>(py+1, px) - map_info_.dist_field.at<float>(py-1, px);
                      
                      float len = std::sqrt(dx*dx + dy*dy);
                      if (len > 1e-3) {
                          obs_dx = dx / len;
                          obs_dy = -dy / len; 
                      }
                }

                curr.x += (alpha * smooth_dx + beta * obs_dx) * learning_rate;
                curr.y += (alpha * smooth_dy + beta * obs_dy) * learning_rate;

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

    vector<Point> smooth_final_path(const vector<Point>& path) {
        vector<Point> smoothed = path;
        int n = path.size();
        
        for (int iter = 0; iter < 50; iter++) {
            for (int i = 1; i < n - 1; i++) {
                smoothed[i].x = 0.25 * smoothed[i-1].x + 0.5 * smoothed[i].x + 0.25 * smoothed[i+1].x;
                smoothed[i].y = 0.25 * smoothed[i-1].y + 0.5 * smoothed[i].y + 0.25 * smoothed[i+1].y;
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

    void generate_path_msg(const vector<Point>& path) {
        stored_path_msg_.header.frame_id = "map";
>>>>>>> minimumlaps
        for (const auto& p : path) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = p.x;
            pose.pose.position.y = p.y;
<<<<<<< HEAD
            pose.pose.orientation.w = 1.0; // 방향 초기화
=======
            pose.pose.orientation.w = 1.0; 
>>>>>>> minimumlaps
            stored_path_msg_.poses.push_back(pose);
        }
    }

    double get_curvature(Point p1, Point p2, Point p3) {
        double D = 2 * (p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y));
<<<<<<< HEAD
        if (std::abs(D) < 1e-5) return 0.0; // 직선인 경우

        double dist_12 = std::hypot(p1.x - p2.x, p1.y - p2.y);
        double dist_23 = std::hypot(p2.x - p3.x, p2.y - p3.y);
        double dist_31 = std::hypot(p3.x - p1.x, p3.y - p1.y);

        // 외접원 반지름 R = (abc) / |D|
        double R = (dist_12 * dist_23 * dist_31) / std::abs(D);
        
        return 1.0 / R; // 곡률 = 1/R
=======
        if (std::abs(D) < 1e-5) return 0.0;
        double dist_12 = std::hypot(p1.x - p2.x, p1.y - p2.y);
        double dist_23 = std::hypot(p2.x - p3.x, p2.y - p3.y);
        double dist_31 = std::hypot(p3.x - p1.x, p3.y - p1.y);
        double R = (dist_12 * dist_23 * dist_31) / std::abs(D);
        return 1.0 / R; 
>>>>>>> minimumlaps
    }

    std::vector<double> generate_velocity_profile(const std::vector<Point>& path) {
        RCLCPP_INFO(this->get_logger(), "Step 4: Generating Velocity Profile...");
<<<<<<< HEAD
        
        size_t n = path.size();
        std::vector<double> velocity(n, 0.0);

        // --- 차량 물리 파라미터 (F1TENTH 차량 기준 튜닝 필요) ---
        double mu = 0.5;          // 타이어 마찰 계수 (높을수록 코너를 빨리 돔)
        double g = 9.81;          // 중력 가속도
        double max_v = 7.0;       // 차량의 절대 최대 속도 (m/s)
        double max_accel = 3.0;   // 최대 가속도 (m/s^2)
        double max_decel = 2.0;   // 최대 감속도 (m/s^2) - 브레이크 성능

        // 1단계: 각 지점의 곡률 기반 최대 속도 계산 (Physical Limit)
        for (size_t i = 1; i < n - 1; i++) {
            double k = get_curvature(path[i-1], path[i], path[i+1]);
            
            if (k < 1e-3) {
                velocity[i] = max_v; // 직선이면 최고 속도
            } else {
                double v_limit = std::sqrt((mu * g) / k);
                velocity[i] = std::min(max_v, v_limit); // 물리 한계와 차량 한계 중 작은 값
            }
        }
        velocity[0] = velocity[n-1] = 2.0; // 시작/끝 속도 안전하게 설정

        // 2단계: 감속 패스 (Backward Pass) - 브레이킹 존 계산
        // 미래의 급커브를 위해 현재 미리 속도를 줄여야 함
        // v_i = sqrt(v_{i+1}^2 + 2 * a * d)
        for (int i = n - 2; i >= 0; i--) {
            double dist = std::hypot(path[i+1].x - path[i].x, path[i+1].y - path[i].y);
            double max_reachable_velocity = std::sqrt(std::pow(velocity[i+1], 2) + 2 * max_decel * dist);
            velocity[i] = std::min(velocity[i], max_reachable_velocity);
        }

        // 3단계: 가속 패스 (Forward Pass) - 가속 구간 계산
        // 현재 속도에서 물리적으로 도달 가능한 최대 속도 제한
        for (size_t i = 1; i < n; i++) {
            double dist = std::hypot(path[i].x - path[i-1].x, path[i].y - path[i-1].y);
            double max_reachable_velocity = std::sqrt(std::pow(velocity[i-1], 2) + 2 * max_accel * dist);
            velocity[i] = std::min(velocity[i], max_reachable_velocity);
        }

        RCLCPP_INFO(this->get_logger(), "Velocity Profile Generated. Max Speed: %.2f m/s", 
            *std::max_element(velocity.begin(), velocity.end()));
        
        return velocity;
    }

    vector<Point> prune_unsafe_points(const vector<Point>& path, double safe_margin_meters) {
        vector<Point> safe_path;
        int pruned_count = 0;
        
        for (const auto& p : path) {
            // 1. 월드 좌표(m) -> 픽셀 좌표(px) 변환
            int px = (int)((p.x - map_info_.origin_x) / map_info_.resolution);
            // Y축 반전 주의
            int py = (int)(map_info_.height - (p.y - map_info_.origin_y) / map_info_.resolution);
            
            // 2. 맵 범위 체크
            if (px >= 0 && px < map_info_.width && py >= 0 && py < map_info_.height) {
                // Distance Field에서 해당 픽셀의 값(벽까지의 픽셀 거리) 가져오기
                float dist_pixel = map_info_.dist_field.at<float>(py, px);
                double dist_meter = dist_pixel * map_info_.resolution; // 미터 단위로 변환
                
                // 3. 안전 거리 이상인 점만 남김
                if (dist_meter >= safe_margin_meters) {
                    safe_path.push_back(p);
                } else {
                    pruned_count++;
                }
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Pruned %d unsafe points (Margin: %.2fm). Remaining: %zu", 
            pruned_count, safe_margin_meters, safe_path.size());
            
        return safe_path;
    }

=======
        size_t n = path.size();
        std::vector<double> velocity(n, 0.0);

        double mu = 0.5;       
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

>>>>>>> minimumlaps
    double get_dist_to_wall(Point p) {
        int px = (int)((p.x - map_info_.origin_x) / map_info_.resolution);
        int py = (int)(map_info_.height - (p.y - map_info_.origin_y) / map_info_.resolution);
        if (px >= 0 && px < map_info_.width && py >= 0 && py < map_info_.height) {
            return map_info_.dist_field.at<float>(py, px) * map_info_.resolution;
        }
        return 0.0;
    }

    void save_to_csv(const vector<Point>& path, const vector<double>& speeds, string filename) {
        ofstream file(filename);
<<<<<<< HEAD
        file << "# x, y, speed_mps, dist_to_wall\n"; // 헤더
        
        for (size_t i = 0; i < path.size(); i++) {
            // 실제 벽과의 거리 계산
            double dist = get_dist_to_wall(path[i]);
            
=======
        file << "# x, y, speed_mps, dist_to_wall\n"; 
        for (size_t i = 0; i < path.size(); i++) {
            double dist = get_dist_to_wall(path[i]);
>>>>>>> minimumlaps
            file << path[i].x << "," << path[i].y << "," << speeds[i] << "," << dist << "\n";
        }
        file.close();
        RCLCPP_INFO(this->get_logger(), "Saved raceline to %s", filename.c_str());
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OfflinePlanner>());
    rclcpp::shutdown();
    return 0;
}