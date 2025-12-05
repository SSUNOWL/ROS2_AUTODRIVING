#!/bin/bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# ==========================================
# 실험 설정
# ==========================================

# 1. Mux Weight 조합 (w_speed w_track w_comfort w_clearance w_dynamics)
# 예: "Speed중시", "안전중시", "추종중시", "밸런스"
mux_params=(
    "1.0 1.0 1.0 1.0 1.0"   # Baseline (Balance)
    "2.0 0.5 0.5 0.5 0.5"   # Speed Heavy
    "0.5 2.0 0.5 0.5 0.5"   # Tracking Heavy
    "0.5 0.5 1.0 2.0 1.0"   # Safety (Clearance) Heavy
    "0.5 0.5 2.0 0.5 1.0"   # Comfort Heavy
)

# 2. Racing Scenario Maps (Solo)
racing_maps=("Spielberg" "hairpin_combo")

# 3. Obstacle Scenario Map & Opponents (Duo)
obs_map="playground"
opponent_csvs=(
    "bumper_slow_1.csv"
    "bumper_v_3.csv"
    # 필요한 만큼 상대 차량 CSV 추가 (시간 관계상 2개만 예시)
)

# ==========================================
# 유틸리티 함수
# ==========================================
cleanup_ros_nodes() {
    echo "   >>> [Cleanup] Killing ROS nodes..."
    pkill -f "map_gym_bridge"
    pkill -f "robot_state_publisher"
    pkill -f "rviz2"
    pkill -f "nav2"
    pkill -f "mux_controller"
    pkill -f "mux_auto_run.launch.py"
    
    # 이제 이 파일 하나만 끄면 됨
    pkill -f "mux_auto_map.launch.py"
    
    pkill -f "racecar_frenet_cpp"
    pkill -f "f1tenth_planner"
    sleep 2
}

# ==========================================
# [Phase 1] Racing Scenario (Spielberg, Hairpin)
# ==========================================
echo "=== Phase 1: Racing Scenarios (Run Logger) ==="

# 이전 프로세스 정리
cleanup_ros_nodes

for map_name in "${racing_maps[@]}"; do
    
    # 확장자 설정
    if [ "$map_name" == "Spielberg" ]; then
        MAP_EXT=".png"
    else
        MAP_EXT=".pgm"
    fi

    echo "------------------------------------------------"
    echo ">>> Starting Map: $map_name (1 Agent)"
    
    # Map Launch (Frenet Map Launch 재사용 - num_agent=1)
    ros2 launch racecar_experiments mux_auto_map.launch.py \
        map_name:=$map_name \
        map_img_ext:=$MAP_EXT \
        use_rviz:=true &
    echo ">>> Waiting 15s for Map initialization..."
    sleep 15
    
    # Weight 별 실험 수행
    for params in "${mux_params[@]}"; do
        read -r ws wt wco wcl wd <<< "$params"
        
        echo ""
        echo "   >>> [Mux Racing] Map=$map_name | Weights: S=$ws T=$wt C=$wco CL=$wcl D=$wd"
        
        ros2 launch racecar_experiments mux_auto_run.launch.py \
            map_name:=$map_name \
            w_speed:=$ws \
            w_track:=$wt \
            w_comfort:=$wco \
            w_clearance:=$wcl \
            w_dynamics:=$wd
            
        echo "   >>> Episode finished. Cooldown..."
        sleep 3
    done
    
    # 다음 맵으로 넘어가기 전 맵 노드 종료
    cleanup_ros_nodes
    sleep 5
done

# ==========================================
# [Phase 2] Obstacle Scenario (Playground)
# ==========================================
echo "=== Phase 2: Obstacle Scenarios (Avoid Logger) ==="

cleanup_ros_nodes

# Playground Map 실행 (FGM Map Launch 재사용 - num_agent=2)
echo "------------------------------------------------"
echo ">>> Starting Map: $obs_map (2 Agents)"

ros2 launch racecar_experiments mux_auto_map.launch.py \
    map_name:=$obs_map \
    map_img_ext:=".pgm" \
    use_rviz:=true &
echo ">>> Waiting 15s for Map initialization..."
sleep 15

# Opponent CSV 별 루프
for opp_csv in "${opponent_csvs[@]}"; do
    
    echo "================================================"
    echo ">>> Opponent CSV: $opp_csv"
    
    # Weight 별 실험 수행
    for params in "${mux_params[@]}"; do
        read -r ws wt wco wcl wd <<< "$params"
        
        echo ""
        echo "   >>> [Mux Obstacle] Opp=$opp_csv | Weights: S=$ws T=$wt C=$wco CL=$wcl D=$wd"
        
        ros2 launch racecar_experiments mux_auto_run.launch.py \
            map_name:=$obs_map \
            opponent_csv_filename:=$opp_csv \
            w_speed:=$ws \
            w_track:=$wt \
            w_comfort:=$wco \
            w_clearance:=$wcl \
            w_dynamics:=$wd
            
        echo "   >>> Episode finished. Cooldown..."
        sleep 3
    done
done

# 전체 종료
echo "=== All Mux Experiments Completed ==="
cleanup_ros_nodes