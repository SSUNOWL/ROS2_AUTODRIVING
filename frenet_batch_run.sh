#!/bin/bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# 지도 목록
maps=("Spielberg")

# RViz 실행 여부 (장시간 학습 시 false 권장)
USE_RVIZ="true"

# 파라미터 리스트 (5 Params: Frenet[Max, Tgt, Acc, Crv] | PP[Max])
params_list=(
    # --- Group 1: Speed 4.5 ---
    "4.5 4.5 5.0 0.8 4.5" 
    "4.5 4.5 5.0 1.0 4.5" 
    "4.5 4.5 5.0 1.0 5.5"
    "4.5 4.5 7.0 0.8 4.5" 
    "4.5 4.5 7.0 1.0 4.5" 
    "4.5 4.5 9.0 1.0 5.5" 

    # --- Group 2: Speed 5.0 ---
    "5.0 5.0 5.0 0.8 5.0" "5.0 5.0 5.0 0.8 6.0"
    "5.0 5.0 5.0 1.0 5.0" "5.0 5.0 5.0 1.0 6.0"
    "5.0 5.0 5.0 1.2 5.0" "5.0 5.0 5.0 1.5 5.0"
    "5.0 5.0 7.0 1.0 5.0" "5.0 5.0 7.0 1.0 6.0"
    "5.0 5.0 9.0 1.0 5.0" "5.0 5.0 9.0 1.0 6.0"
    "5.0 4.0 7.0 1.0 5.0"

    # --- Group 3: Speed 5.5 ---
    "5.5 5.5 5.0 0.8 5.5" "5.5 5.5 5.0 0.8 6.5"
    "5.5 5.5 5.0 1.0 5.5" "5.5 5.5 5.0 1.0 6.5"
    "5.5 5.5 5.0 1.2 5.5" "5.5 5.5 5.0 1.5 5.5"
    "5.5 5.5 7.0 1.0 5.5" "5.5 5.5 7.0 1.0 6.5"
    "5.5 5.5 9.0 1.0 5.5" "5.5 5.5 9.0 1.0 6.5"

    # --- Group 4: Speed 6.0 ---
    "6.0 6.0 5.0 0.8 6.0" "6.0 6.0 5.0 0.8 7.0"
    "6.0 6.0 5.0 1.0 6.0" "6.0 6.0 5.0 1.0 7.0"
    "6.0 6.0 5.0 1.2 6.0" "6.0 6.0 5.0 1.2 7.0"
    "6.0 6.0 5.0 1.5 6.0"
    "6.0 6.0 7.0 1.0 6.0" "6.0 6.0 7.0 1.0 7.0"
    "6.0 6.0 7.0 1.2 6.0" "6.0 6.0 7.0 1.2 7.0"
    "6.0 6.0 9.0 1.0 6.0" "6.0 6.0 9.0 1.0 7.0"
    "6.0 5.0 7.0 1.0 6.0"

    # --- Group 5: Speed 6.5 ---
    "6.5 6.5 5.0 0.8 6.5" "6.5 6.5 5.0 0.8 7.5"
    "6.5 6.5 5.0 1.0 6.5" "6.5 6.5 5.0 1.0 7.5"
    "6.5 6.5 5.0 1.2 6.5" "6.5 6.5 5.0 1.2 7.5"
    "6.5 6.5 7.0 1.0 6.5" "6.5 6.5 7.0 1.0 7.5"
    "6.5 6.5 7.0 1.2 6.5" "6.5 6.5 7.0 1.2 7.5"
    "6.5 6.5 9.0 1.0 6.5" "6.5 6.5 9.0 1.0 7.5"

    # --- Group 6: Speed 7.0 ---
    "7.0 7.0 5.0 1.0 7.0" "7.0 7.0 5.0 1.0 8.0"
    "7.0 7.0 5.0 1.2 7.0" "7.0 7.0 5.0 1.2 8.0"
    "7.0 7.0 7.0 1.0 7.0" "7.0 7.0 7.0 1.0 8.0"
    "7.0 7.0 7.0 1.2 7.0" "7.0 7.0 7.0 1.2 8.0"
    "7.0 7.0 7.0 1.5 7.0"
    "7.0 7.0 9.0 1.0 7.0" "7.0 7.0 9.0 1.0 8.0"
    "7.0 7.0 9.0 1.2 7.0" "7.0 7.0 9.0 1.2 8.0"

    # --- Group 7: Speed 7.5 ---
    "7.5 7.5 5.0 1.0 7.5" "7.5 7.5 5.0 1.2 7.5"
    "7.5 7.5 7.0 1.0 7.5" "7.5 7.5 7.0 1.0 8.5"
    "7.5 7.5 7.0 1.2 7.5" "7.5 7.5 7.0 1.2 8.5"
    "7.5 7.5 9.0 1.0 7.5" "7.5 7.5 9.0 1.0 8.5"
    "7.5 7.5 9.0 1.2 7.5" "7.5 7.5 9.0 1.2 8.5"

    # --- Group 8: Speed 8.0 ---
    "8.0 8.0 5.0 1.0 8.0" "8.0 8.0 5.0 1.2 8.0"
    "8.0 8.0 7.0 1.0 8.0" "8.0 8.0 7.0 1.0 9.0"
    "8.0 8.0 7.0 1.2 8.0" "8.0 8.0 7.0 1.2 9.0"
    "8.0 8.0 7.0 1.5 8.0"
    "8.0 8.0 9.0 1.0 8.0" "8.0 8.0 9.0 1.0 9.0"
    "8.0 8.0 9.0 1.2 8.0" "8.0 8.0 9.0 1.2 9.0"

    # --- Group 9: Speed 8.5 ---
    "8.5 8.5 7.0 1.0 8.5" "8.5 8.5 7.0 1.2 8.5"
    "8.5 8.5 9.0 1.0 8.5" "8.5 8.5 9.0 1.2 8.5"
    "8.5 8.5 9.0 1.2 9.5"

    # --- Group 10: Speed 9.0 ---
    "9.0 9.0 7.0 1.2 9.0" "9.0 9.0 7.0 1.5 9.0"
    "9.0 9.0 9.0 1.2 9.0" "9.0 9.0 9.0 1.2 10.0"
)

# 이전 잔여 프로세스 정리 함수
cleanup_ros_nodes() {
    echo "   >>> Cleaning up old ROS nodes..."
    pkill -f "map_gym_bridge"
    pkill -f "gym_bridge"
    pkill -f "map_server"
    pkill -f "lifecycle_manager"
    pkill -f "robot_state_publisher"
    pkill -f "rviz2"
    pkill -f "frenet_auto_map.launch.py"
    pkill -f "collision_monitor"
    pkill -f "run_logger"
    pkill -f "static_path_publisher"
}

echo "=== Frenet Experiment Pipeline (Headless Mode) ==="

# 시작 전 한 번 청소
cleanup_ros_nodes
sleep 2

for map_name in "${maps[@]}"; do
    
    if [ "$map_name" == "Spielberg" ]; then
        MAP_EXT=".png"
    else
        MAP_EXT=".pgm"
    fi

    echo "------------------------------------------------"
    echo ">>> Starting MAP: $map_name (Ext: $MAP_EXT)"
    
    # [수정됨] use_rviz 인자 추가
    ros2 launch racecar_experiments frenet_auto_map.launch.py \
        map_name:=$map_name \
        map_img_ext:=$MAP_EXT \
        use_rviz:=$USE_RVIZ &
    
    echo ">>> Map launching... Waiting 15s..."
    sleep 15
    
    count=1
    total_runs=${#params_list[@]}

    for params in "${params_list[@]}"; do
        read -r max_s target_s max_a max_c pp_s <<< "$params"
        
        echo ""
        echo "   >>> [$count / $total_runs] Running Frenet:"
        echo "       Frenet[Max:$max_s Tgt:$target_s Acc:$max_a Crv:$max_c] | PP[Max:$pp_s]"
        
        ros2 launch racecar_experiments frenet_auto_run.launch.py \
            map_name:=$map_name \
            max_speed:=$max_s \
            target_speed:=$target_s \
            max_accel:=$max_a \
            max_curvature:=$max_c \
            pp_max_speed:=$pp_s
            
        echo "   >>> Run finished. Cooling down (3s)..."
        sleep 3
        ((count++))
    done
    
    echo ">>> Experiment set finished for $map_name."
    echo ">>> Killing Map Nodes..."
    
    cleanup_ros_nodes
    sleep 10
    
done

echo "=== All Experiments Completed ==="