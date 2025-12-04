#!/bin/bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# 지도 목록
maps=("hairpin_combo" "Spielberg")

# 파라미터 리스트
params_list=(
    "5.5 4.6 5.0 0.90"
    "5.5 4.6 5.0 0.95"
    "5.5 4.6 5.0 1.00"
    "5.5 4.6 6.0 0.90"
    "5.5 4.6 6.0 0.95"
    "5.5 4.6 6.0 1.00"
    "5.5 4.8 5.0 0.90"
    "5.5 4.8 5.0 0.95"
    "5.5 4.8 5.0 1.00"
    "5.5 4.8 6.0 0.90"
    "5.5 4.8 6.0 0.95"
    "5.5 4.8 6.0 1.00"
    "5.5 5.0 5.0 0.90"
    "5.5 5.0 5.0 0.95"
    "5.5 5.0 5.0 1.00"
    "5.5 5.0 6.0 0.90"
    "5.5 5.0 6.0 0.95"
    "5.5 5.0 6.0 1.00"
    "5.5 5.2 5.0 0.90"
    "5.5 5.2 5.0 0.95"
    "5.5 5.2 6.0 0.90"
    "5.5 5.2 6.0 0.95"
    "6.0 4.6 5.0 0.90"
    "6.0 4.6 5.0 0.95"
    "6.0 4.6 5.0 1.00"
    "6.0 4.6 6.0 0.90"
    "6.0 4.6 6.0 0.95"
    "6.0 4.6 6.0 1.00"
    "6.0 4.8 5.0 0.90"
    "6.0 4.8 5.0 0.95"
    "6.0 4.8 5.0 1.00"
    "6.0 4.8 6.0 0.90"
    "6.0 4.8 6.0 0.95"
    "6.0 4.8 6.0 1.00"
    "6.0 5.0 5.0 0.90"
    "6.0 5.0 5.0 0.95"
    "6.0 5.0 5.0 1.00"
    "6.0 5.0 6.0 0.90"
    "6.0 5.0 6.0 0.95"
    "6.0 5.0 6.0 1.00"
    "6.0 5.2 5.0 0.90"
    "6.0 5.2 5.0 0.95"
    "6.0 5.2 6.0 0.90"
    "6.0 5.2 6.0 0.95"
    "6.5 4.6 5.0 0.90"
    "6.5 4.6 5.0 0.95"
    "6.5 4.6 5.0 1.00"
    "6.5 4.6 6.0 0.90"
    "6.5 4.6 6.0 0.95"
    "6.5 4.6 6.0 1.00"
    "6.5 4.8 5.0 0.90"
    "6.5 4.8 5.0 0.95"
    "6.5 4.8 5.0 1.00"
    "6.5 4.8 6.0 0.90"
    "6.5 4.8 6.0 0.95"
    "6.5 4.8 6.0 1.00"
    "6.5 5.0 5.0 0.90"
    "6.5 5.0 5.0 0.95"
    "6.5 5.0 5.0 1.00"
    "6.5 5.0 6.0 0.90"
    "6.5 5.0 6.0 0.95"
    "6.5 5.0 6.0 1.00"
    "6.5 5.2 5.0 0.90"
    "6.5 5.2 5.0 0.95"
    "6.5 5.2 6.0 0.90"
    "6.5 5.2 6.0 0.95"
)

# 이전 잔여 프로세스 정리 함수
cleanup_ros_nodes() {
    echo "   >>> Cleaning up old ROS nodes..."
    # 지도 관련 노드들 강제 종료
    pkill -f "map_gym_bridge"
    pkill -f "gym_bridge"
    pkill -f "map_server"
    pkill -f "lifecycle_manager"
    pkill -f "robot_state_publisher"
    pkill -f "rviz2"
    # [수정됨] 변경된 map launch 파일 이름으로 종료
    pkill -f "frenet_auto_map.launch.py"
}

echo "=== Frenet Experiment Pipeline (Robust Cleanup) ==="

# 시작 전 한 번 청소
cleanup_ros_nodes
sleep 2

for map_name in "${maps[@]}"; do
    
    # 확장자 자동 결정
    if [ "$map_name" == "Spielberg" ]; then
        MAP_EXT=".png"
    else
        MAP_EXT=".pgm"
    fi

    echo "------------------------------------------------"
    echo ">>> Starting MAP: $map_name (Ext: $MAP_EXT)"
    
     if [ "$map_name" == "hairpin_combo" ]; then
        params_list=(
            "5.5 4.6 6.0 0.90"
            "5.5 4.6 6.0 0.95"
            "5.5 4.6 6.0 1.00"
            "5.5 4.8 5.0 0.95"
            "5.5 5.0 5.0 0.90"
            "5.5 5.0 6.0 0.90"
            "5.5 5.0 6.0 0.95"
            "5.5 5.2 5.0 0.90"
            "5.5 5.2 5.0 0.95"
            "5.5 5.2 6.0 0.90"
            "5.5 5.2 6.0 0.95"
            "6.0 4.6 5.0 0.90"
            "6.0 4.6 5.0 0.95"
            "6.0 4.6 5.0 1.00"
            "6.0 4.6 6.0 0.95"
            "6.0 5.0 5.0 0.90"
            "6.0 5.0 5.0 0.95"
            "6.0 5.0 5.0 1.00"
            "6.0 5.0 6.0 0.95"
            "6.0 5.2 5.0 0.90"
            "6.0 5.2 5.0 0.95"
            "6.0 5.2 6.0 0.90"
            "6.0 5.2 6.0 0.95"
            "6.5 4.6 5.0 0.90"
            "6.5 4.6 5.0 0.95"
            "6.5 4.6 5.0 1.00"
            "6.5 4.6 6.0 0.90"
            "6.5 4.6 6.0 0.95"
            "6.5 5.0 5.0 0.90"
            "6.5 5.0 5.0 0.95"
            "6.5 5.0 5.0 1.00"
            "6.5 5.0 6.0 0.90"
            "6.5 5.0 6.0 0.95"
            "6.5 5.2 6.0 0.90"
            "6.5 5.2 6.0 0.95"
        )
    elif [ "$map_name" == "Spielberg" ]; then
        params_list=(
            "5.5 4.8 5.0 0.95"
            "5.5 4.8 5.0 1.00"
            "5.5 4.8 6.0 0.90"
            "5.5 5.2 5.0 0.90"
            "5.5 5.2 6.0 0.90"
            "5.5 5.2 6.0 0.95"
            "6.0 4.6 6.0 0.95"
            "6.0 4.8 5.0 0.90"
            "6.0 5.0 6.0 1.00"
            "6.0 5.2 5.0 0.95"
            "6.5 4.6 5.0 0.90"
            "6.5 4.8 5.0 0.95"
            "6.5 4.8 5.0 1.00"
            "6.5 4.8 6.0 0.90"
            "6.5 4.8 6.0 1.00"
            "6.5 4.8 6.0 0.95"
        )
    fi
    # [수정됨] frenet_auto_map.launch.py 실행
    ros2 launch racecar_experiments frenet_auto_map.launch.py \
        map_name:=$map_name \
        map_img_ext:=$MAP_EXT &
    
    # PID 저장은 하지만, 종료는 pkill로 할 예정
    MAP_PID=$!
    
    echo ">>> Map launching... Waiting 15s for full initialization..."
    sleep 15
    
    for params in "${params_list[@]}"; do
        read -r max_s target_s max_a max_c <<< "$params"
        
        echo ""
        echo "   >>> Running Frenet Experiment: Params=[$params]"
        
        # [수정됨] frenet_auto_run.launch.py 실행
        ros2 launch racecar_experiments frenet_auto_run.launch.py \
            map_name:=$map_name \
            max_speed:=$max_s \
            target_speed:=$target_s \
            max_accel:=$max_a \
            max_curvature:=$max_c
            
        echo "   >>> Finished. Cooling down (3s)..."
        sleep 3
    done
    
    echo ">>> Experiment set finished for $map_name."
    echo ">>> Killing Map Nodes..."
    
    # [핵심] 강력한 종료 로직
    cleanup_ros_nodes
    
    # 프로세스들이 완전히 죽을 때까지 대기
    sleep 10
    
done

echo "=== All Frenet Experiments Completed ==="