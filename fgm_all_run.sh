#!/bin/bash
export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/humble/setup.bash
source install/setup.bash

# ==========================================
# [설정] FGM 파라미터 리스트
# (기존 fgm_batch_run.sh의 파라미터 구조 사용)
# ==========================================
params_list=(
    "0.8900 0.4550 183.4000 25.3000 0.5350 0.5900 7.6000 0.0610 1.7000 0.1980 0.5050 0.1520 5.5"
    # 필요한 파라미터 셋을 여기에 계속 추가하세요.
    #"0.9222 0.4516 183.8757 22.0032 0.4220 0.4703 7.6272 0.0624 1.7476 0.1864 0.5780 0.1593 4.3227"
)

# ==========================================
# [설정] 맵 및 시나리오
# ==========================================
racing_maps=("Spielberg" "hairpin_combo") 
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

USE_RVIZ="true"

# ==========================================
# [함수] 프로세스 정리
# ==========================================
cleanup_run_nodes() {
    echo "   >>> [Partial Cleanup] Killing Controller & Loggers..."
    pkill -f "fgm_unified_run.launch.py"
    pkill -f "fgm_node"
    pkill -f "pure_pursuit_node"
    pkill -f "opponent_pure_pursuit_node"
    pkill -f "avoid_logger"
    pkill -f "collision_monitor"
    pkill -f "static_path_publisher"
    sleep 3
}

cleanup_all_nodes() {
    echo "   >>> [Full Cleanup] Killing EVERYTHING..."
    cleanup_run_nodes
    
    pkill -f "mux_auto_map.launch.py"
    pkill -f "map_gym_bridge"
    pkill -f "map_server"
    pkill -f "gym_bridge"
    pkill -f "robot_state_publisher"
    pkill -f "rviz2"
    
    echo "   >>> Waiting for RViz to close..."
    count=0
    while pgrep -f "rviz2" > /dev/null; do
        sleep 1
        count=$((count+1))
        if [ $count -ge 3 ]; then
            pkill -9 -f "rviz2"
            break
        fi
    done
    
    ros2 daemon stop > /dev/null 2>&1
    ros2 daemon start > /dev/null 2>&1
    echo "   >>> Cleanup Complete."
    sleep 2
}

# ==========================================
# [Phase 1] Racing Scenarios (Spielberg, Hairpin)
# ==========================================
echo "=== Phase 1: Racing Scenarios (FGM) ==="

cleanup_all_nodes

for map_name in "${racing_maps[@]}"; do
    
    if [ "$map_name" == "Spielberg" ]; then MAP_EXT=".png"; else MAP_EXT=".pgm"; fi

    echo "------------------------------------------------"
    echo ">>> [Map Setup] Starting Map: $map_name"
    
    ros2 launch racecar_experiments mux_auto_map.launch.py \
        map_name:=$map_name \
        map_img_ext:=$MAP_EXT \
        use_rviz:=$USE_RVIZ &
    
    echo ">>> Waiting 15s for Map & RViz initialization..."
    sleep 15
    
    for params in "${params_list[@]}"; do
        read -r p1 p2 p3 p4 p5 p6 p7 p8 p9 p10 p11 p12 p13 <<< "$params"
        
        echo ""
        echo "   >>> [Run] FGM Racing: Gap=$p1, Speed=$p13 ..."
        
        ros2 launch racecar_experiments fgm_unified_run.launch.py \
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
            
        cleanup_run_nodes
    done
    
    echo ">>> All runs for $map_name done."
    cleanup_all_nodes

done

# ==========================================
# [Phase 2] Obstacle Scenarios (Playground)
# ==========================================
echo "=== Phase 2: Obstacle Scenarios (FGM) ==="

cleanup_all_nodes

echo "------------------------------------------------"
echo ">>> [Map Setup] Starting Map: $obs_map"

# Playground (차량 2대 자동 생성)
ros2 launch racecar_experiments mux_auto_map.launch.py \
    map_name:=$obs_map \
    map_img_ext:=".pgm" \
    use_rviz:=$USE_RVIZ &

echo ">>> Waiting 15s for Map & RViz initialization..."
sleep 15

for opp_csv in "${opponent_csvs[@]}"; do
    echo "================================================"
    echo ">>> Opponent: $opp_csv"
    echo "================================================"
    
    for params in "${params_list[@]}"; do
        read -r p1 p2 p3 p4 p5 p6 p7 p8 p9 p10 p11 p12 p13 <<< "$params"
        
        echo ""
        echo "   >>> [Run] FGM vs $opp_csv (Params: Gap=$p1)..."
        
        # opponent_csv_filename 인자 추가
        ros2 launch racecar_experiments fgm_unified_run.launch.py \
            map_name:=$obs_map \
            opponent_csv_filename:=$opp_csv \
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
            
        cleanup_run_nodes
    done
done

echo "=== All Experiments Completed ==="
cleanup_all_nodes