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
    # Set 1: 기준값 느낌 (지금까지 쓰던 기본형)
    # Gap=1.2, Bub=0.5, Clr=0.55, WW=0.6, AW=6.0, SW=0.08, HB=2.0, CT=0.25, SA=0.5, DBS=0.12, SPD=4.0
    "1.2 0.5 180.0 25.0 0.55 0.6 6.0 0.08 2.0 0.25 0.5 0.12 4.0"

    # Set 2: 조금 더 보수적 + 속도 3.5 (넓은 안전거리 + 낮은 속도)
    # Gap=1.3, Bub=0.55, Clr=0.60, WW=0.5, AW=5.0, SW=0.10, HB=2.5, CT=0.30, SA=0.4, DBS=0.15, SPD=3.5
    "1.3 0.55 180.0 30.0 0.60 0.5 5.0 0.10 2.5 0.30 0.4 0.15 3.5"

    # Set 3: 꽤 공격적인 세트 (속도↑, angle 강하게, steer penalty↓)
    # head-on에서 “그래도 피해주냐” 테스트용
    # Gap=1.0, Bub=0.4, Clr=0.45, WW=0.7, AW=8.0, SW=0.05, HB=1.5, CT=0.20, SA=0.6, DBS=0.18, SPD=5.5
    "1.0 0.4 180.0 25.0 0.45 0.7 8.0 0.05 1.5 0.20 0.6 0.18 5.5"

    # Set 4: 아주 안전 위주 (버블 크고 클리어런스 크게, 속도 낮게)
    # Gap=1.5, Bub=0.7, Clr=0.75, WW=0.4, AW=4.0, SW=0.13, HB=3.0, CT=0.35, SA=0.3, DBS=0.22, SPD=3.0
    "1.5 0.7 190.0 35.0 0.75 0.4 4.0 0.13 3.0 0.35 0.3 0.22 3.0"

    # Set 5: 폭 넓은 갭 선호 + 직진 집착 (AW↑, WW↑), 중간 속도
    # Gap=1.1, Bub=0.45, Clr=0.50, WW=0.9, AW=7.0, SW=0.07, HB=1.0, CT=0.22, SA=0.7, DBS=0.10, SPD=4.5
    "1.1 0.45 175.0 22.0 0.50 0.9 7.0 0.07 1.0 0.22 0.7 0.10 4.5"

    # Set 6: 히스테리시스 강하게 (gap 자주 안 바꾸는 세트)
    # Gap=1.2, Bub=0.5, Clr=0.60, WW=0.6, AW=6.0, SW=0.09, HB=3.5, CT=0.40, SA=0.5, DBS=0.12, SPD=4.0
    "1.2 0.5 185.0 28.0 0.60 0.6 6.0 0.09 3.5 0.40 0.5 0.12 4.0"

    # Set 7: smoothing 강하고 dynamic bubble 크게 (속도 의존 버블↑)
    # Gap=1.3, Bub=0.6, Clr=0.55, WW=0.5, AW=5.5, SW=0.08, HB=2.0, CT=0.25, SA=0.75, DBS=0.25, SPD=4.2
    "1.3 0.6 180.0 25.0 0.55 0.5 5.5 0.08 2.0 0.25 0.75 0.25 4.2"

    # Set 8: FOV 좁게 두고 앞만 잘 보는 스타일 + 중간보다 살짝 높은 속도
    # Gap=1.0, Bub=0.5, Clr=0.50, WW=0.7, AW=6.5, SW=0.06, HB=2.0, CT=0.20, SA=0.6, DBS=0.15, SPD=5.0
    "1.0 0.5 160.0 20.0 0.50 0.7 6.5 0.06 2.0 0.20 0.6 0.15 5.0"
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