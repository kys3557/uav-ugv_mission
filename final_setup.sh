#!/bin/bash

#=======================================================================
# ✨ 종료 시그널 핸들러 (사용자님의 안정적인 버전으로 교체) ✨
#=======================================================================
cleanup() {
    echo ""
    echo ">>> Stopping all processes..."

    # 1. 백그라운드 잡(pid) 목록을 가져와서 SIGINT(Ctrl+C) 신호 전송
    for pid in $(jobs -p); do
        kill -SIGINT "$pid" >/dev/null 2>&1 || true
    done

    # 2. 모든 백그라운드 잡이 실제로 종료될 때까지 대기
    wait

    echo "Cleanup complete."
    exit 0
}

# 스크립트가 종료 신호(SIGINT, SIGTERM)를 받으면 cleanup 함수를 실행하도록 설정
trap cleanup SIGINT SIGTERM EXIT


#=======================================================================
# ✨ 실행 로직 (시간 기반 대기 버전) ✨
#=======================================================================

# 1. PX4 SITL 및 Gazebo 실행
echo "[1/3] Starting PX4 SITL and Gazebo..."
(
  cd ~/PX4-Autopilot_ASP && \
  make px4_sitl gz_x500_gimbal
) &

# 고정된 시간만큼 대기
echo "Waiting 10 seconds for simulation to initialize..."
sleep 10

# 2. QGroundControl 실행
echo "[2/3] Starting QGroundControl..."
(
  cd ~ && \
  ./QGroundControl.AppImage
) &

# 3. ROS 2 launch 파일 실행
echo "[3/3] Launching ROS 2 nodes..."
(
  source /opt/ros/humble/setup.bash && \
  cd ~/ros2_ws && \
  source install/setup.bash && \
  ros2 launch gazebo_env_setup turn_interfaces.launch.py
) &


# 모든 백그라운드 프로세스가 종료될 때까지 스크립트가 대기
echo ""
echo "============================================="
echo "All systems launched. Press Ctrl+C to terminate all."
echo "============================================="
wait