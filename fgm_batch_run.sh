#!/bin/bash
export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/humble/setup.bash
source install/setup.bash

# ==========================================
# [설정] FGM 실험 파라미터 (13개 변수)
# ==========================================
# p1: Gap Threshold
# p2: Bubble Radius
# p3: FOV Angle
# p4: Speed Check FOV
# p5: Required Clearance
# p6: Width Weight
# p7: Angle Weight
# p8: Steer Weight
# p9: Hysteresis Bonus
# p10: Change Threshold
# p11: Smoothing Alpha
# p12: Dynamic Bubble Speed Coeff
# p13: Max Speed
params_list=(
    "0.8900 0.4550 183.4000 25.3000 0.5350 0.5900 7.6000 0.0610 1.7000 0.1980 0.5050 0.1520 4.4000"
    # 추가 파라미터들을 여기에 작성하세요
    # "0.9222 0.4516 183.8757 22.0032 0.4220 0.4703 7.6272 0.0624 1.7476 0.1864 0.5780 0.1593 4.3227"
)

# 1. Racing Maps (장애물 없는 맵)
racing_maps=("Spielberg" "hairpin_combo")

# 2. Obstacle Map & CSVs (장애물 있는 맵)
obs_map="playground"
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

# RViz 사용 여부
USE_RVIZ="true"

# ==========================================
# [함수] 강력한 프로세스 정리 (RViz 포함)
# ==========================================

# 1. 가벼운 정리 (같은 맵에서 파라미터만 바꿀 때)
cleanup_run_nodes() {
    echo "   >>> [Partial Cleanup] Killing FGM & Loggers..."
    pkill -f "fgm_auto_run.launch.py"
    pkill -f "fgm_node"
    pkill -f "pure_pursuit"
    pkill -f "pure_pursuit_node"
    pkill -f "opponent_pure_pursuit_node"
    pkill -f "static_path_publisher"
    pkill -f "collision_monitor"
    pkill -f "avoid_logger"
    pkill -f "run_logger"
    
    sleep 3
}

# 2. 완전 박멸 (맵 바꿀 때 - RViz까지 종료)
cleanup_all_nodes() {
    echo "   >>> [Full Cleanup] Killing EVERYTHING (Map, RViz, Sim)..."
    
    # 먼저 주행 노드 정리
    cleanup_run_nodes
    
    # 맵 & 시뮬레이터 & RViz 종료 명령
    pkill -f "fgm_auto_map.launch.py"
    pkill -f "map_gym_bridge"
    pkill -f "map_server"
    pkill -f "gym_bridge"
    pkill -f "nav2_map_server"
    pkill -f "lifecycle_manager"
    pkill -f "nav2_lifecycle_manager"
    pkill -f "robot_state_publisher"
    pkill -f "nav2"
    
    # RViz 종료 명령
    pkill -f "rviz2"

    # Map Server가 죽을 때까지 대기
    echo "   >>> Waiting for map_server to die..."
    while pgrep -f "map_server" > /dev/null; do
        sleep 1
    done

    # RViz가 죽을 때까지 대기 (가장 중요)
    echo "   >>> Waiting for RViz to close..."
    count=0
    while pgrep -f "rviz2" > /dev/null; do
        echo "       ... RViz is still running (wait 1s)"
        sleep 1
        count=$((count+1))
        
        # 5초 이상 안 꺼지면 강제 종료
        if [ $count -ge 5 ]; then
            echo "       !!! Force Killing RViz (SIGKILL) !!!"
            pkill -9 -f "rviz2"
            break
        fi
    done
    
    # ROS 2 데몬 재시작 (통신 오류 방지)
    echo "   >>> Resetting ROS 2 Daemon..."
    ros2 daemon stop > /dev/null 2>&1
    ros2 daemon start > /dev/null 2>&1
    
    echo "   >>> Cleanup Complete."
    sleep 5
}

# ==========================================
# [Phase 1] Racing Scenarios (단독 주행)
# ==========================================
echo "=== Phase 1: Racing Scenarios (Solo FGM) ==="

# 시작 전 정리
cleanup_all_nodes

for map_name in "${racing_maps[@]}"; do
    
    if [ "$map_name" == "Spielberg" ]; then MAP_EXT=".png"; else MAP_EXT=".pgm"; fi

    echo "------------------------------------------------"
    echo ">>> [Map Setup] Starting Map: $map_name"
    
    # 맵 & RViz 실행 (백그라운드)
    ros2 launch racecar_experiments fgm_auto_map.launch.py \
        map_name:=$map_name \
        map_img_ext:=$MAP_EXT \
        use_rviz:=$USE_RVIZ &
    
    # 맵 로드 대기
    echo ">>> Waiting 15s for Map & RViz initialization..."
    sleep 15
    
    for params in "${params_list[@]}"; do
        read -r p1 p2 p3 p4 p5 p6 p7 p8 p9 p10 p11 p12 p13 <<< "$params"
        
        echo ""
        echo "   >>> [Run Phase 1] Map: $map_name"
        echo "   >>> Params: Gap=$p1, Speed=$p13 ..."
        
        # 상대 차량(csv) 없이 단독 주행
        # (launch 파일에서 opponent_csv_filename이 없으면 자동으로 단독 주행 모드라고 가정)
        ros2 launch racecar_experiments fgm_auto_run.launch.py \
            map_name:=$map_name \
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
            
        echo "   >>> Run Finished. Cooldown..."
        cleanup_run_nodes
    done
    
    # 다음 맵을 위해 완전 정리
    echo ">>> All runs for $map_name done."
    cleanup_all_nodes

done

# ==========================================
# [Phase 2] Obstacle Scenarios (대항차 주행)
# ==========================================
echo "=== Phase 2: Obstacle Scenarios (FGM + Opponent) ==="

# (Phase 1 끝난 직후 이미 cleanup이 되었지만 안전을 위해 확인)
# cleanup_all_nodes 

echo "------------------------------------------------"
echo ">>> [Map Setup] Starting Map: $obs_map"

ros2 launch racecar_experiments fgm_auto_map.launch.py \
    map_name:=$obs_map \
    map_img_ext:=".pgm" \
    use_rviz:=$USE_RVIZ &

echo ">>> Waiting 15s for Map & RViz initialization..."
sleep 15

for opp_csv in "${opponent_csvs[@]}"; do
    echo "================================================"
    echo ">>> Target Opponent CSV: $opp_csv"
    echo "================================================"

    for params in "${params_list[@]}"; do
        read -r p1 p2 p3 p4 p5 p6 p7 p8 p9 p10 p11 p12 p13 <<< "$params"
        
        echo ""
        echo "   >>> [Run Phase 2] Opponent: $opp_csv"
        echo "   >>> Params: Gap=$p1, Speed=$p13 ..."
        
        # 상대 차량 포함 주행
        ros2 launch racecar_experiments fgm_auto_run.launch.py \
            map_name:=$obs_map \
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
            
        echo "   >>> Episode finished. Cooling down..."
        cleanup_run_nodes
    done
done

echo "=== All Experiments Completed ==="
cleanup_all_nodes