#!/bin/bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# ==========================================
# 실험 설정
# ==========================================

# 1. Mux Weight 조합 (w_speed w_track w_comfort w_clearance w_dynamics)
mux_params=(
    "1.0 1.0 1.0 1.0 1.0"   # Baseline (Balance)
    # 필요한 만큼 추가
)

# 2. Racing Scenario Maps (Solo)
racing_maps=("Spielberg" "hairpin_combo")

# 3. Obstacle Scenario Map & Opponents (Duo)
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

# RViz 사용 여부 (배치 돌릴때는 false 권장, 필요시 true)
USE_RVIZ="true"

# ==========================================
# 유틸리티 함수 (강력한 프로세스 킬러)
# ==========================================
cleanup_all_nodes() {
    echo "   >>> [Cleanup] Killing ALL ROS nodes (Map & Simulation)..."
    
    # 1. 실행 중인 런치 파일들 종료
    pkill -f "mux_auto_run.launch.py"
    pkill -f "mux_auto_map.launch.py"
    
    # 2. 핵심 바이너리 강제 종료
    pkill -f "map_server"
    pkill -f "nav2_map_server"
    pkill -f "lifecycle_manager"
    pkill -f "nav2_lifecycle_manager"
    pkill -f "map_gym_bridge"
    pkill -f "robot_state_publisher"
    pkill -f "rviz2"
    pkill -f "nav2"
    pkill -f "mux_controller"
    pkill -f "racecar_frenet_cpp"
    pkill -f "f1tenth_planner"
    pkill -f "run_logger"
    pkill -f "avoid_logger"
    pkill -f "collision_monitor"

    # 3. Map Server가 완전히 죽을 때까지 대기 (Frenet 스크립트 참고)
    echo "   >>> Waiting for map_server to terminate..."
    COUNT=0
    while pgrep -f "map_server" > /dev/null; do
        sleep 1
        COUNT=$((COUNT+1))
        if [ $COUNT -ge 10 ]; then
            echo "   !!! Force killing map_server !!!"
            pkill -9 -f "map_server"
            break
        fi
    done
    
    # RViz도 확인 사살
    pkill -9 -f "rviz2"

    echo "   >>> Cleanup Complete."
    sleep 3  # 프로세스 정리 후 안전 대기 시간
}

# ==========================================
# [Phase 1] Racing Scenario (Spielberg, Hairpin)
# ==========================================
echo "=== Phase 1: Racing Scenarios (Run Logger) ==="

# 최초 시작 전 정리
cleanup_all_nodes

for map_name in "${racing_maps[@]}"; do
    
    # 확장자 설정
    if [ "$map_name" == "Spielberg" ]; then
        MAP_EXT=".png"
    else
        MAP_EXT=".pgm"
    fi

    echo "------------------------------------------------"
    echo ">>> Starting Map: $map_name (1 Agent)"
    
    # [Map Launch] 백그라운드 실행 (&)
    # 이 노드는 내부 루프(Weight 실험)가 도는 동안 계속 살아있어야 함
    ros2 launch racecar_experiments mux_auto_map.launch.py \
        map_name:=$map_name \
        map_img_ext:=$MAP_EXT \
        use_rviz:=$USE_RVIZ &
    
    # Frenet 실험 때처럼 맵이 로드되고 안정화될 때까지 충분히 기다림
    echo ">>> Waiting 15s for Map initialization..."
    sleep 15
    
    # Weight 별 실험 수행 (Inner Loop)
    for params in "${mux_params[@]}"; do
        read -r ws wt wco wcl wd <<< "$params"
        
        echo ""
        echo "   >>> [Mux Racing] Map=$map_name | Weights: S=$ws T=$wt C=$wco CL=$wcl D=$wd"
        
        # [Run Launch] 포그라운드 실행 (이게 끝날 때까지 스크립트 대기)
        # mux_auto_run은 실험이 끝나면(Crash/Finish) 스스로 종료됨
        ros2 launch racecar_experiments mux_auto_run.launch.py \
            map_name:=$map_name \
            w_speed:=$ws \
            w_track:=$wt \
            w_comfort:=$wco \
            w_clearance:=$wcl \
            w_dynamics:=$wd
            
        echo "   >>> Episode finished. Cooldown (3s)..."
        sleep 3
    done
    
    # [중요] 한 맵의 모든 실험이 끝났으므로, 다음 맵을 위해 모든 노드를 정리함
    echo ">>> Finished experiments for $map_name. Cleaning up..."
    cleanup_all_nodes
    sleep 5 # 다음 맵 시작 전 안전 마진
done

# ==========================================
# [Phase 2] Obstacle Scenario (Playground)
# ==========================================
echo "=== Phase 2: Obstacle Scenarios (Avoid Logger) ==="

# 혹시 모를 잔여 프로세스 정리
cleanup_all_nodes

# Playground Map 실행 (한 번만 실행하고 유지)
echo "------------------------------------------------"
echo ">>> Starting Map: $obs_map (2 Agents)"

ros2 launch racecar_experiments mux_auto_map.launch.py \
    map_name:=$obs_map \
    map_img_ext:=".pgm" \
    use_rviz:=$USE_RVIZ &

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
            
        echo "   >>> Episode finished. Cooldown (3s)..."
        sleep 3
    done
done

# 전체 종료
echo "=== All Mux Experiments Completed ==="
cleanup_all_nodes