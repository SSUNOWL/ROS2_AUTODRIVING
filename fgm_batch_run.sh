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

params_list=(
    "0.9222 0.4516 183.8757 22.0032 0.4220 0.4703 7.6272 0.0624 1.7476 0.1864 0.5780 0.1593 4.3227"
    "0.8102 0.4324 195.6872 24.8976 0.4616 0.5619 7.5198 0.0631 1.6547 0.2196 0.5333 0.1449 4.1789"
    "0.8987 0.4345 187.1440 24.7017 0.5857 0.5079 7.2406 0.0598 1.8362 0.2181 0.5351 0.1399 4.0164"
    "0.9078 0.3976 188.8425 25.4270 0.5093 0.5299 7.6821 0.0685 1.7813 0.2109 0.5429 0.1535 4.5172"
    "0.8373 0.3985 158.5752 23.3862 0.4890 0.5878 7.0270 0.0690 1.5144 0.1946 0.5470 0.1570 4.5447"
    "0.8378 0.4488 190.8925 23.3304 0.4923 0.4714 6.8655 0.0593 1.8196 0.2023 0.5698 0.1449 4.3545"
    "0.9018 0.3978 188.4835 23.4061 0.4855 0.5090 6.8277 0.0665 1.6843 0.2002 0.5608 0.1554 4.5289"
    "0.8645 0.3633 178.0857 23.1873 0.4421 0.4694 6.9477 0.0660 1.6232 0.1939 0.5961 0.1457 4.5537"
    "0.8039 0.4086 166.0933 23.6056 0.5336 0.4525 7.8124 0.0652 1.6668 0.1769 0.5431 0.1436 4.3800"
    "0.8533 0.3856 179.8138 27.0754 0.5072 0.5342 8.3709 0.0683 1.8000 0.2234 0.4886 0.1423 3.9813"
    "0.9352 0.4469 177.6807 22.5136 0.4720 0.5421 7.7906 0.0651 1.9716 0.2442 0.5217 0.1400 3.6495"
    "0.8604 0.4327 187.3279 23.6546 0.5290 0.6403 7.6955 0.0602 1.5592 0.1715 0.4854 0.1513 5.0223"
    "0.8545 0.4119 185.3114 27.2891 0.4937 0.5558 8.8941 0.0570 1.8140 0.1837 0.5426 0.1567 4.3449"
    "0.9218 0.4384 182.8019 23.9747 0.4825 0.5297 7.7049 0.0687 1.7218 0.1961 0.5165 0.1679 4.4772"
    "0.9205 0.5109 177.3709 27.5391 0.5428 0.5322 6.7682 0.0691 1.9344 0.1952 0.4895 0.1698 4.0103"
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