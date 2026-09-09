#!/usr/bin/env bash
# 로봇/시뮬레이터를 "실행 상태로 올리는" 스크립트 (= bringup).
#
#   ./scripts/06_bringup.sh mock      물리 없이 배선만 (가장 먼저)
#   ./scripts/06_bringup.sh mujoco    MuJoCo 물리 시뮬레이션 + RViz
#   ./scripts/06_bringup.sh real      실물 SO-101 (집에서)
#   ./scripts/06_bringup.sh mujoco use_rviz:=false      추가 인자는 그대로 전달된다
#
# 이 터미널은 실행이 끝날 때까지 계속 점유된다. 종료는 Ctrl+C.
# 다른 명령(04 점검, 05 GUI, group_goal_client)은 반드시 새 터미널에서 실행한다.
set -eo pipefail

MODE="${1:-}"
case "$MODE" in
  mock|mujoco|real) shift ;;
  *) sed -n '2,12p' "$0"; exit 2 ;;
esac

WS="${SO101_WS:-$HOME/so101_ws}"
[ -f "$WS/install/setup.bash" ] || { echo "워크스페이스가 없다. 먼저 ./scripts/01_setup_workspace.sh" >&2; exit 1; }

# ROS setup.bash 는 set -u 와 호환되지 않는다
set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u

# ---------------------------------------------------------------- WSL 대비 설정
# 두 가지 WSL 특유의 문제를 여기서 미리 막는다.
#  (1) networkingMode=mirrored 인 WSL 은 127.0.0.1 통신 자체가 죽어 있을 수 있다.
#      lib_dds.sh 가 그 경우에만 127.0.0.1 을 피하는 DDS 프로필을 만들어 export 한다.
#  (2) Fast DDS 의 공유메모리 전송은 강제 종료된 프로세스가 /dev/shm 에 세그먼트를
#      남기면 통신을 조용히 막는다. 그래서 (1)이 아닐 때는 UDP 전용을 기본으로 쓴다.
# 두 증상 모두 로그에는 아래 한 줄의 반복으로 나타난다.
#   [controller_manager]: Waiting for data on 'robot_description' topic ...
# 다른 설정을 쓰려면 이 스크립트를 부르기 전에 직접 export 하면 그것을 존중한다.
# shellcheck source=scripts/lib_dds.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib_dds.sh"
so101_dds_setup || true
if [ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ] \
   && [ -z "${FASTDDS_BUILTIN_TRANSPORTS:-}" ] && [ -z "${RMW_IMPLEMENTATION:-}" ]; then
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
fi

# 실행 중인 ROS 프로세스가 없을 때만 고아 세그먼트를 정리한다(있으면 건드리지 않는다).
if ! pgrep -f 'ros2_control_node|move_group|robot_state_publisher' >/dev/null 2>&1; then
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
fi

cat <<TXT
=============================================================
 bringup: hardware_type:=$MODE
 전송: RMW=${RMW_IMPLEMENTATION:-기본} FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-기본}
 이 터미널은 로봇이 켜져 있는 동안 계속 점유된다. 종료 = Ctrl+C
 점검·제어는 새 터미널에서:
   ./scripts/04_smoke_test.sh
   ./scripts/05_gui.sh sliders
   ros2 run so101_project group_goal_client.py --group arm --named rest

 정상 기동이면 몇 초 안에 아래가 보인다:
   Loaded hardware 'SO101_${MODE}...' from plugin ...
   Configured and activated joint_state_broadcaster / arm_controller / gripper_controller
 만약 다음이 계속 반복되면 통신이 막힌 것이다 (정상 아님):
   [controller_manager]: Waiting for data on 'robot_description' topic ...
   → Ctrl+C 후 새 터미널에서  ./scripts/07_doctor.sh --fix
=============================================================
TXT

exec ros2 launch so101_project bringup.launch.py "hardware_type:=$MODE" "$@"
