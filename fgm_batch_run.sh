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
    "1.1621 0.4998 167.1159 26.2867 0.5428 0.6162 6.6756 0.0806 2.1030 0.2090 0.4981 0.1119 3.6100"
    "1.2879 0.4469 167.3546 24.3318 0.5513 0.5768 5.8427 0.0864 2.0861 0.2767 0.4938 0.1133 3.9284"
    "1.0959 0.5036 183.4920 25.3531 0.5600 0.7208 6.9009 0.0745 1.9540 0.2207 0.4764 0.1230 4.3859"
    "0.9939 0.4935 169.5012 23.6917 0.5033 0.5746 5.5791 0.0794 1.7188 0.2207 0.5852 0.1076 3.6490"
    "1.0875 0.4853 206.4516 30.8101 0.5650 0.6830 5.5263 0.0784 2.1244 0.2587 0.4850 0.1187 3.5600"
    "1.0186 0.3822 176.5703 24.4672 0.4670 0.7567 8.0995 0.0514 1.5064 0.2000 0.5654 0.1846 5.4572"
    "1.0309 0.3756 210.0000 28.1467 0.4100 0.7667 8.1682 0.0519 1.2906 0.2148 0.6218 0.1640 5.2925"
    "0.9766 0.3967 183.7975 25.1049 0.4409 0.7085 8.9418 0.0397 1.4716 0.2028 0.6142 0.1746 4.7270"
    "0.8773 0.4276 184.7231 28.4547 0.4382 0.6966 7.3261 0.0487 1.6560 0.2093 0.6832 0.1970 5.6932"
    "1.0662 0.3905 205.1127 25.8780 0.4524 0.6609 8.6333 0.0453 1.5939 0.1969 0.6562 0.1908 6.0000"
    "0.9946 0.3625 190.5232 21.8559 0.4313 0.7846 8.4080 0.0472 1.3784 0.2005 0.5416 0.1703 5.6373"
    "0.8167 0.4097 196.6365 26.2175 0.4526 0.7232 9.0344 0.0417 1.4291 0.2095 0.5241 0.2013 5.6621"
    "1.0651 0.4342 192.1908 23.8581 0.4584 0.7131 8.1730 0.0465 1.4823 0.1976 0.6184 0.1944 5.0342"
    "0.9405 0.3737 178.1999 27.9629 0.4573 0.7473 8.0073 0.0553 1.6028 0.2135 0.6266 0.2135 5.4097"
    "0.9634 0.4035 151.1493 28.2085 0.4971 0.6103 7.1989 0.0436 1.4047 0.2070 0.6252 0.1840 4.8784"
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
    use_rviz:=false &

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