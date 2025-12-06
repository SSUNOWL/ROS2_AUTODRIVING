#!/bin/bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# ==========================================
# [설정] 실험 파라미터
# ==========================================

# 1. Mux Weight 91개
mux_params=(
    "1.0 1.0 1.0 1.0 1.0 1.0"
#   # ===== 1) Balanced (정석형) 18개 =====
#   "0.8841 1.3866 0.8504 1.3767 1.0629 0.2115"
#   "1.0714 1.4109 1.5262 1.552 1.4207 0.1955"
#   "1.1381 1.1881 1.4511 1.595 1.5983 0.1877"
#   "1.2333 1.1929 1.4518 1.0927 1.379 0.1851"
#   "0.9035 1.3472 1.102 1.634 1.4355 0.1901"
#   "1.2575 1.531 1.2576 1.2109 1.6963 0.1841"
#   "1.2886 1.3851 1.0061 1.5466 1.5632 0.2146"
#   "1.2244 1.5734 1.4106 1.2118 1.6116 0.2167"
#   "1.3487 1.6967 1.1989 1.5631 1.1507 0.2064"
#   "1.31 1.3208 1.2002 1.0344 1.2855 0.1921"
#   "1.2043 1.1211 1.3884 1.4308 1.2563 0.202"
#   "0.9449 1.2976 1.439 1.0401 1.1516 0.187"
#   "1.0939 1.6234 0.9082 1.0446 1.2508 0.2149"
#   "1.0885 1.5078 1.4127 1.4364 1.5588 0.1816"
#   "1.524 1.4754 1.3739 1.1054 1.2128 0.191"
#   "1.1698 1.5535 1.306 1.2074 1.3573 0.1941"
#   "0.9569 1.3932 1.4536 1.5237 1.1112 0.1848"
#   "0.9084 1.4964 1.0915 1.5146 1.2503 0.1965"

#   # ===== 2) Aggressive (공격형) 18개 =====
#   "2.316 1.7818 0.5751 0.719 0.6781 0.1492"
#   "2.0411 1.0494 0.8521 0.4032 0.7938 0.1544"
#   "2.9131 1.7269 0.9444 0.699 0.5896 0.1355"
#   "2.908 1.1838 0.7348 0.4507 0.9684 0.1402"
#   "2.3654 1.7615 0.6445 0.6267 0.5972 0.1301"
#   "2.1567 1.3737 0.698 0.8333 0.9677 0.1378"
#   "2.0847 1.5993 0.8473 0.412 0.6453 0.1327"
#   "2.9772 1.4183 0.9777 0.5069 0.9121 0.1288"
#   "2.9819 1.4202 0.4224 0.4294 0.4704 0.1345"
#   "2.4932 1.3465 0.5484 0.6115 0.353 0.1539"
#   "2.7608 1.1883 0.5954 0.3531 0.7395 0.1251"
#   "2.9497 1.6935 0.4106 0.7772 0.5144 0.1512"
#   "2.3556 1.2047 0.8122 0.3048 0.8075 0.1359"
#   "2.3225 1.7967 0.9319 0.923 0.985 0.1398"
#   "2.8861 1.236 0.8526 0.5532 0.3505 0.1633"
#   "2.6482 1.2062 0.9791 0.9266 0.9076 0.158"
#   "2.7545 1.3992 0.4503 0.3347 0.397 0.1608"
#   "2.5922 1.2238 0.8772 0.6338 0.9725 0.1328"

#   # ===== 3) Defensive (수비형) 18개 =====
#   "0.5138 0.8112 0.9638 2.6165 2.0187 0.2588"
#   "0.7571 1.2629 0.9884 2.2419 2.6276 0.2356"
#   "0.8861 1.0509 1.1419 2.7173 2.9842 0.2586"
#   "0.9376 0.9327 0.9817 2.2676 2.4136 0.2489"
#   "0.7979 1.559 0.9723 2.9099 2.5774 0.2722"
#   "0.6928 1.1848 1.3157 2.794 2.3887 0.2309"
#   "0.6147 1.4843 0.9283 2.6025 2.8232 0.2735"
#   "0.5638 0.8564 0.8468 2.8497 2.6366 0.2548"
#   "0.605 1.3506 0.7183 2.8088 2.7902 0.249"
#   "0.6806 0.9693 0.9278 2.1538 2.3257 0.2478"
#   "0.3983 1.2881 1.0924 2.3659 2.991 0.2417"
#   "0.9271 1.007 0.9742 2.2519 2.1258 0.272"
#   "1.0388 1.5873 1.2429 3.0026 2.642 0.2454"
#   "0.5529 1.0906 1.0699 2.8004 2.1754 0.2415"
#   "0.9615 1.3358 1.149 2.6532 2.9927 0.2332"
#   "0.5131 1.4804 1.0786 2.7189 2.8385 0.2532"
#   "0.8967 1.2108 1.1897 2.028 2.3758 0.2582"
#   "0.5053 0.8522 1.4159 2.7225 2.5519 0.2394"

#   # ===== 4) Comfort (승차감 중시) 18개 =====
#   "1.128 1.2613 3.1432 1.8351 1.6502 0.2074"
#   "0.7262 1.1648 2.3669 1.4363 1.6912 0.2277"
#   "1.2874 1.1571 3.1134 1.6453 0.9524 0.2152"
#   "1.2112 1.5546 2.517 1.2644 1.3001 0.1987"
#   "1.189 0.9277 2.6455 1.9303 1.5129 0.1799"
#   "1.3203 1.2512 2.432 1.2317 1.6035 0.2085"
#   "0.7025 1.5402 2.4927 1.6393 1.0657 0.1871"
#   "1.1741 1.5918 3.167 1.4674 1.5063 0.1959"
#   "1.0142 1.0977 2.0636 1.6002 1.2406 0.2008"
#   "1.0341 1.0204 2.05 1.8391 1.0403 0.222"
#   "1.29 1.4675 2.6409 1.5904 1.371 0.1825"
#   "1.3316 1.0386 2.7936 1.5558 1.7769 0.2013"
#   "0.8228 1.0491 3.0837 1.9735 1.783 0.1975"
#   "1.1751 1.1714 2.2522 1.1078 1.7631 0.2059"
#   "1.0613 1.3086 3.073 1.4748 1.6143 0.221"
#   "1.3487 1.3079 2.1699 1.1159 1.0643 0.1869"
#   "0.6663 0.9267 2.921 1.3054 1.4095 0.2134"
#   "0.8282 1.5812 2.5276 1.8827 1.0338 0.2171"

#   # ===== 5) TrackKeeping (라인고집형) 18개 =====
#   "1.0391 3.0619 1.3841 1.0825 1.2592 0.2152"
#   "1.4696 2.0526 0.9807 1.9658 1.719 0.1768"
#   "1.2497 2.4833 1.4393 1.494 1.5434 0.2193"
#   "1.3967 2.6396 0.8537 1.6959 1.4198 0.1989"
#   "1.5232 2.2839 1.1423 1.2898 1.2642 0.1799"
#   "1.4717 3.1207 1.2259 1.485 1.1865 0.205"
#   "1.4516 2.7474 1.0353 1.2191 1.035 0.1973"
#   "1.0608 2.3754 0.8409 1.7021 1.6963 0.2147"
#   "1.5381 2.4013 1.3278 1.3426 1.2767 0.2187"
#   "1.1052 2.3484 1.1856 1.7559 1.343 0.185"
#   "1.9337 2.2162 0.9466 1.5956 1.6827 0.187"
#   "1.6218 3.0615 0.7573 1.9687 1.1672 0.2108"
#   "1.1311 2.9123 1.428 2.1387 1.9753 0.191"
#   "1.2082 3.0446 1.3749 1.1709 1.1202 0.1757"
#   "1.3175 2.1451 1.1442 1.1301 1.2104 0.2152"
#   "1.182 2.5179 0.7391 1.2932 1.4873 0.1875"
#   "1.0241 2.4336 1.3423 1.2577 1.4355 0.1752"
#   "1.9219 2.1553 1.3408 1.8841 1.3669 0.1919"
)



# 2. Maps
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

# [핵심] RViz를 켜고 싶으므로 true로 설정
USE_RVIZ="true"

# ==========================================
# [함수] 강력한 프로세스 정리 (RViz 포함)
# ==========================================

# 1. 가벼운 정리 (같은 맵에서 실험만 다시 할 때)
cleanup_run_nodes() {
    echo "   >>> [Partial Cleanup] Killing Controller & Loggers..."
    pkill -f "mux_auto_run.launch.py"
    pkill -f "mux_controller"
    pkill -f "planner_mux_node"
    pkill -f "pure_pursuit"
    pkill -f "racecar_frenet_cpp"
    pkill -f "fgm_node"
    pkill -f "f1tenth_planner"
    pkill -f "run_logger"
    pkill -f "avoid_logger"
    pkill -f "collision_monitor"
    pkill -f "static_path_publisher"
    # 주행 노드는 금방 죽으므로 짧게 대기
    sleep 2
}

# 2. 완전 박멸 (맵 바꿀 때 - RViz까지 확인 사살)
cleanup_all_nodes() {
    echo "   >>> [Full Cleanup] Killing EVERYTHING (Map, RViz, Sim)..."
    
    # 먼저 주행 노드 정리
    cleanup_run_nodes
    
    # 맵 & 시뮬레이터 & RViz 종료 명령
    pkill -f "mux_auto_map.launch.py"
    pkill -f "map_gym_bridge"
    pkill -f "map_server"
    pkill -f "gym_bridge"
    pkill -f "nav2_map_server"
    pkill -f "lifecycle_manager"
    pkill -f "nav2_lifecycle_manager"
    pkill -f "robot_state_publisher"
    pkill -f "nav2"
    
    # [핵심 1] RViz 종료 명령
    pkill -f "rviz2"

    # [핵심 2] Map Server가 죽을 때까지 대기
    echo "   >>> Waiting for map_server to die..."
    while pgrep -f "map_server" > /dev/null; do
        sleep 1
    done

    # [핵심 3] RViz가 죽을 때까지 대기 (가장 중요)
    # [핵심 3] RViz가 죽을 때까지 대기 (소프트웨어 렌더링용 강력 버전)
    echo "   >>> Waiting for RViz to close..."
    count=0
    while pgrep -f "rviz2" > /dev/null; do
        echo "       ... RViz is still running (wait 1s)"
        sleep 1
        count=$((count+1))
        
        # [수정] 10초는 너무 깁니다. 3초만 지나도 안 꺼지면 바로 강제 종료(-9) 시킵니다.
        if [ $count -ge 3 ]; then
            echo "       !!! Force Killing RViz (SIGKILL) !!!"
            pkill -9 -f "rviz2"  # -9 옵션이 핵심입니다 (즉시 사망)
            break
        fi
    done
    
    # [핵심 4] ROS 2 데몬 재시작 (통신 찌꺼기 제거)
    # 맵이 겹치는 현상은 DDS Discovery 캐시 문제일 수 있음
    echo "   >>> Resetting ROS 2 Daemon..."
    ros2 daemon stop > /dev/null 2>&1
    ros2 daemon start > /dev/null 2>&1
    
    echo "   >>> Cleanup Complete."
    sleep 5
}

# ==========================================
# [Phase 1] Racing Scenarios
# ==========================================
echo "=== Phase 1: Racing Scenarios ==="

cleanup_all_nodes

for map_name in "${racing_maps[@]}"; do
    
    if [ "$map_name" == "Spielberg" ]; then MAP_EXT=".png"; else MAP_EXT=".pgm"; fi

    echo "------------------------------------------------"
    echo ">>> [Map Setup] Starting Map: $map_name"
    
    # 맵 & RViz 실행 (백그라운드)
    ros2 launch racecar_experiments mux_auto_map.launch.py \
        map_name:=$map_name \
        map_img_ext:=$MAP_EXT \
        use_rviz:=$USE_RVIZ &
    
    # RViz가 켜지고 맵이 로드될 때까지 충분히 대기
    echo ">>> Waiting 15s for Map & RViz initialization..."
    sleep 15
    
    for params in "${mux_params[@]}"; do
        read -r ws wt wco wcl wd dm <<< "$params"
        echo ""
        echo "   >>> [Run] Weights: S=$ws T=$wt..."
        
        ros2 launch racecar_experiments mux_auto_run.launch.py \
            map_name:=$map_name \
            w_speed:=$ws w_track:=$wt w_comfort:=$wco w_clearance:=$wcl w_dynamics:=$wd d_min:=$dm
            
        echo "   >>> Run Finished. Cooldown..."
        
        # [Partial Cleanup] 지도는 끄지 않고 주행 노드만 끔
        cleanup_run_nodes
    done
    
    # 맵 다 썼으니 완전 종료 (RViz가 여기서 꺼짐)
    echo ">>> All runs for $map_name done."
    cleanup_all_nodes

done

# ==========================================
# [Phase 2] Obstacle Scenarios
# ==========================================
echo "=== Phase 2: Obstacle Scenarios ==="

cleanup_all_nodes

echo "------------------------------------------------"
echo ">>> [Map Setup] Starting Map: $obs_map"

ros2 launch racecar_experiments mux_auto_map.launch.py \
    map_name:=$obs_map \
    map_img_ext:=".pgm" \
    use_rviz:=$USE_RVIZ &

echo ">>> Waiting 15s for Map & RViz initialization..."
sleep 15

for opp_csv in "${opponent_csvs[@]}"; do
    echo ">>> Opponent: $opp_csv"
    for params in "${mux_params[@]}"; do
        read -r ws wt wco wcl wd dm <<< "$params"
        
        ros2 launch racecar_experiments mux_auto_run.launch.py \
            map_name:=$obs_map \
            opponent_csv_filename:=$opp_csv \
            w_speed:=$ws w_track:=$wt w_comfort:=$wco w_clearance:=$wcl w_dynamics:=$wd d_min:=$dm
            
        cleanup_run_nodes
    done
done

echo "=== All Experiments Completed ==="
cleanup_all_nodes