#!/bin/bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# 지도 설정
MAP_NAME="playground"
# [수정됨] playground 맵은 .pgm 확장자를 사용합니다.
MAP_EXT=".pgm"

# 상대 차량 경로 파일 (8가지)
opponent_csvs=(
    "bumper_slow_0.5.csv"
    "bumper_slow_1.csv"
    "bumper_slow_1.5.csv"
    "bumper_slow_2.csv"
    "bumper_slow_2.5.csv"
    "bumper_slow_3.csv"
    "bumper_slow_4.csv"
    "bumper_slow_5.csv"
)

# FGM 및 속도 파라미터 리스트
# 순서: (1~12) FGM Params ... (13) pp_max_speed
# [순서 상세]
# 1. gap_threshold
# 2. bubble_radius
# 3. fov_angle
# 4. speed_check_fov
# 5. required_clearance
# 6. width_weight
# 7. angle_weight
# 8. steer_weight
# 9. hysteresis_bonus
# 10. change_threshold
# 11. smoothing_alpha
# 12. dynamic_bubble_coeff
# 13. [NEW] pp_max_speed

params_list=(
    # Set 1: 기본 설정 + 속도 4.0
    "1.2 0.5 180.0 25.0 0.55 0.6 6.0 0.08 2.0 0.25 0.5 0.12 4.0"
    
    # Set 2: 조금 더 보수적 + 속도 3.5
    "1.3 0.55 180.0 30.0 0.60 0.5 5.0 0.10 2.5 0.30 0.4 0.15 3.5"
)

cleanup_ros_nodes() {
    echo "   >>> Cleaning up ROS nodes..."
    pkill -f "fgm_auto_map.launch.py"
    pkill -f "map_gym_bridge"
    pkill -f "robot_state_publisher"
    pkill -f "rviz2"
    pkill -f "fgm_node"
    pkill -f "pure_pursuit_node"
    pkill -f "opponent_pure_pursuit_node"
    pkill -f "static_path_publisher"
    pkill -f "collision_monitor"
    pkill -f "run_logger"
}

echo "=== FGM + Opponent Experiment Pipeline ==="

# 초기 정리
cleanup_ros_nodes
sleep 2

# 루프 시작: 상대 차량 CSV 별로 수행
for opp_csv in "${opponent_csvs[@]}"; do
    
    echo "------------------------------------------------"
    echo ">>> Starting Environment for Opponent: $opp_csv"
    
    # 맵 실행 (RViz 포함)
    ros2 launch racecar_experiments fgm_auto_map.launch.py \
        map_name:=$MAP_NAME \
        map_img_ext:=$MAP_EXT \
        use_rviz:=true &
    
    echo ">>> Map launching... Waiting 10s..."
    sleep 10
    
    # 파라미터 세트 별 실험
    for params in "${params_list[@]}"; do
        # 13개 변수 읽기 (p13 = pp_max_speed)
        read -r p1 p2 p3 p4 p5 p6 p7 p8 p9 p10 p11 p12 p13 <<< "$params"
        
        echo ""
        echo "   >>> Running FGM Test..."
        echo "   >>> Opponent: $opp_csv"
        echo "   >>> Params: Gap=$p1, Clearance=$p5, Speed=$p13 ..."
        
        # 실행
        ros2 launch racecar_experiments fgm_auto_run.launch.py \
            map_name:=$MAP_NAME \
            opponent_csv_filename:="$opp_csv" \
            fgm_gap_threshold:=$p1 \
            fgm_bubble_radius:=$p2 \
            fgm_fov_angle:=$p3 \
            fgm_speed_check_fov_deg:=$p4 \
            fgm_required_clearance:=$p5 \
            fgm_width_weight:=$p6 \
            fgm_angle_weight:=$p7 \
            fgm_steer_weight:=$p8 \
            fgm_hysteresis_bonus:=$p9 \
            fgm_change_threshold:=$p10 \
            fgm_smoothing_alpha:=$p11 \
            fgm_dynamic_bubble_speed_coeff:=$p12 \
            pp_max_speed:=$p13
            
        echo "   >>> Finished. Cooling down (3s)..."
        sleep 3
    done
    
    echo ">>> Completed set for $opp_csv. Resetting Map..."
    cleanup_ros_nodes
    sleep 5
    
done

echo "=== All Experiments Completed ==="