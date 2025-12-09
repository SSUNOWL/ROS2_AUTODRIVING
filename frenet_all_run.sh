#!/bin/bash
export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/humble/setup.bash
source install/setup.bash

# ==========================================
# [설정] Frenet 파라미터 리스트
# Format: "Max_Speed Target_Speed Max_Accel Max_Curv PP_Speed"
# ==========================================
frenet_params=(
    "6.5 5.2 6.0 0.9 5.5"
    # 필요한 만큼 파라미터 조합을 추가하세요.
    #"5.0 5.0 5.0 1.0 5.0"
    #"6.0 6.0 7.0 1.0 6.0"
)

# ==========================================
# [설정] 맵 및 시나리오
# ==========================================
racing_maps=("Spielberg" "hairpin_combo") # 레이싱 맵
obs_map="playground"                      # 장애물 맵 (상대 차량 스폰)

# 상대 차량 경로 파일 (Playground용)
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

# 시각화 필요 시 true, 아니면 false
USE_RVIZ="true"

# ==========================================
# [함수] 강력한 프로세스 정리
# ==========================================
cleanup_run_nodes() {
    echo "   >>> [Partial Cleanup] Killing Controller & Loggers..."
    pkill -f "frenet_unified_run.launch.py"
    pkill -f "racecar_frenet_cpp"
    pkill -f "pure_pursuit_node"
    pkill -f "opponent_pure_pursuit_node"
    pkill -f "run_logger"
    pkill -f "collision_monitor"
    pkill -f "static_path_publisher"
    sleep 3
}

cleanup_all_nodes() {
    echo "   >>> [Full Cleanup] Killing EVERYTHING..."
    cleanup_run_nodes
    
    pkill -f "mux_auto_map.launch.py" # Mux Map Launch를 사용하므로 이걸 꺼야 함
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
echo "=== Phase 1: Racing Scenarios (Frenet) ==="

cleanup_all_nodes

for map_name in "${racing_maps[@]}"; do
    
    if [ "$map_name" == "Spielberg" ]; then MAP_EXT=".png"; else MAP_EXT=".pgm"; fi

    echo "------------------------------------------------"
    echo ">>> [Map Setup] Starting Map: $map_name"
    
    # [중요] Mux의 Map Launch를 사용하여 1대/2대 로직 자동 처리 (여기선 Racing이라 1대)
    ros2 launch racecar_experiments mux_auto_map.launch.py \
        map_name:=$map_name \
        map_img_ext:=$MAP_EXT \
        use_rviz:=$USE_RVIZ &
    
    echo ">>> Waiting 15s for Map & RViz initialization..."
    sleep 15
    
    for params in "${frenet_params[@]}"; do
        read -r ms ts ma mc pps <<< "$params"
        echo ""
        echo "   >>> [Run] Frenet: Speed=$ms, Target=$ts ..."
        
        # 새로 만든 통합 Launch 파일 실행
        ros2 launch racecar_experiments frenet_unified_run.launch.py \
            map_name:=$map_name \
            max_speed:=$ms \
            target_speed:=$ts \
            max_accel:=$ma \
            max_curvature:=$mc \
            pp_max_speed:=$pps
            
        cleanup_run_nodes
    done
    
    echo ">>> All runs for $map_name done."
    cleanup_all_nodes

done

# ==========================================
# [Phase 2] Obstacle Scenarios (Playground)
# ==========================================
echo "=== Phase 2: Obstacle Scenarios (Frenet) ==="

cleanup_all_nodes

echo "------------------------------------------------"
echo ">>> [Map Setup] Starting Map: $obs_map"

# Playground 맵 실행 (Mux Map Launch가 자동으로 차량 2대를 스폰함)
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
    
    for params in "${frenet_params[@]}"; do
        read -r ms ts ma mc pps <<< "$params"
        
        echo ""
        echo "   >>> [Run] Frenet vs $opp_csv (Speed=$ms)..."
        
        # Playground 모드로 실행 (opponent_csv 입력)
        ros2 launch racecar_experiments frenet_unified_run.launch.py \
            map_name:=$obs_map \
            opponent_csv_filename:=$opp_csv \
            max_speed:=$ms \
            target_speed:=$ts \
            max_accel:=$ma \
            max_curvature:=$mc \
            pp_max_speed:=$pps
            
        cleanup_run_nodes
    done
done

echo "=== All Experiments Completed ==="
cleanup_all_nodes