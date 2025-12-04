#!/bin/bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# 지도 설정
MAP_NAME="playground"
MAP_EXT=".pgm"

# 상대 차량 경로 파일 (8가지)
opponent_csvs=(
    "bumper_slow_0.5.csv"
    "bumper_slow_1.csv"
    "bumper_slow_1.5.csv"
    "bumper_slow_2.csv"
    "bumper_v_2.csv"
    "bumper_v_3.csv"
    "bumper_v_4.csv"
    "bumper_v_5.csv"
)

# FGM 및 속도 파라미터 리스트
# FGM 및 속도 파라미터 리스트
params_list=(
    "0.965 0.445 186.0 24.9 0.540 0.605 7.35 0.063 1.80 0.208 0.515 0.145 4.25"
    "0.875 0.435 180.7 25.5 0.525 0.570 7.00 0.069 1.88 0.218 0.530 0.138 4.10"
    "0.915 0.425 190.0 23.0 0.510 0.550 8.00 0.057 1.65 0.190 0.490 0.158 4.60"
    "0.935 0.460 177.8 26.7 0.560 0.630 6.75 0.071 2.05 0.230 0.525 0.130 4.05"
    "0.895 0.442 185.4 24.1 0.515 0.585 7.28 0.066 1.78 0.205 0.500 0.147 4.35"
    "0.845 0.415 192.3 22.4 0.495 0.520 8.20 0.060 1.62 0.183 0.460 0.165 4.70"
    "0.980 0.455 182.6 25.7 0.545 0.610 7.05 0.068 1.92 0.220 0.510 0.140 4.18"
    "0.900 0.438 188.9 23.8 0.505 0.565 7.85 0.063 1.70 0.195 0.495 0.160 4.50"
    "0.925 0.448 175.5 26.2 0.555 0.600 6.82 0.073 2.00 0.227 0.535 0.128 4.12"
    "0.880 0.428 181.9 25.0 0.520 0.580 7.45 0.067 1.82 0.212 0.510 0.150 4.28"




)



# 모든 노드를 강제로 종료하는 함수 (스크립트 시작/종료 시 사용)
cleanup_all_nodes() {
    echo "   >>> [Cleanup] Killing ALL ROS nodes (Map & Planner)..."
    pkill -f "fgm_auto_map.launch.py"
    pkill -f "map_gym_bridge"
    pkill -f "robot_state_publisher"
    pkill -f "rviz2"
    pkill -f "nav2"
    pkill -f "lifecycle_manager"
    pkill -f "map_server"
    
    # Planner 관련 노드도 종료
    pkill -f "fgm_auto_run.launch.py"
    pkill -f "fgm_node"
    pkill -f "pure_pursuit_node"
    pkill -f "opponent_pure_pursuit_node"
    pkill -f "static_path_publisher"
    pkill -f "collision_monitor"
    pkill -f "avoid_logger"
}

# ------------------------------------------------------------------
# 1. 초기화 및 지도(Sim Environment) 실행 - [한 번만 실행됨]
# ------------------------------------------------------------------
echo "=== FGM + Opponent Experiment Pipeline ==="

# 기존에 켜져 있던 노드 정리
cleanup_all_nodes
sleep 2

echo "------------------------------------------------"
echo ">>> Launching Map & Gym Bridge (Keep Alive)..."

# 맵 실행 (RViz 포함) - 백그라운드(&) 실행
ros2 launch racecar_experiments fgm_auto_map.launch.py \
    map_name:=$MAP_NAME \
    map_img_ext:=$MAP_EXT \
    use_rviz:=true &

# 맵과 Gym Bridge가 완전히 로드될 때까지 충분히 대기 (중요)
echo ">>> Map launching... Waiting 15s for initialization..."
sleep 15

# ------------------------------------------------------------------
# 2. 실험 루프 시작 (상대 차량 CSV -> 파라미터)
# ------------------------------------------------------------------

for opp_csv in "${opponent_csvs[@]}"; do
    
    echo "================================================"
    echo ">>> Target Opponent CSV: $opp_csv"
    echo "================================================"
    
    # 파라미터 세트 별 실험
    for params in "${params_list[@]}"; do
        # 13개 변수 읽기
        read -r p1 p2 p3 p4 p5 p6 p7 p8 p9 p10 p11 p12 p13 <<< "$params"
        
        echo ""
        echo "   >>> [Run] Starting Episode..."
        echo "   >>> Opponent: $opp_csv"
        echo "   >>> Params: Gap=$p1, Speed=$p13 ..."
        
        # ------------------------------------------------------
        # Planner 및 Controller 실행
        # (이 launch 파일이 실행되면 static_path_publisher가 
        #  자차/상대차 위치를 초기화해주므로 지도를 껏다 킬 필요 없음)
        # ------------------------------------------------------
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
            
        # launch 파일은 avoid_logger가 종료되면 자동으로 꺼지도록 설정되어 있음
        # 따라서 여기서 pkill을 할 필요 없이 자연스럽게 넘어감
            
        echo "   >>> [End] Episode finished. Cooling down (3s)..."
        sleep 3
    done
    
    echo ">>> Completed all params for $opp_csv."
    # 여기서는 지도를 끄지 않음 (루프 계속)
    
done

# ------------------------------------------------------------------
# 3. 모든 실험 종료 후 정리
# ------------------------------------------------------------------
echo "=== All Experiments Completed ==="
cleanup_all_nodes