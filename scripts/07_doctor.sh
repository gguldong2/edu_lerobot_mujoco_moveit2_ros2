#!/usr/bin/env bash
# WSL + ROS 2 통신 상태를 진단하고, 필요하면 복구한다.
#
#   ./scripts/07_doctor.sh          진단만 (아무것도 건드리지 않는다)
#   ./scripts/07_doctor.sh --fix    복구까지 (ROS 프로세스 종료 + 고아 SHM 정리 + 데몬 정지)
#
# 언제 쓰나: bringup 로그에 아래가 계속 반복될 때
#   [controller_manager]: Waiting for data on 'robot_description' topic to finish initialization
#   [spawner]: Failed to acquire lock / Could not contact service /controller_manager/list_controllers
# 또는 다른 터미널에서 ros2 node list 가 비어 있거나 TimeoutError 가 날 때.
set -o pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FIX=0
[ "${1:-}" = "--fix" ] && FIX=1

set +u
source /opt/ros/jazzy/setup.bash
[ -f "$HOME/so101_ws/install/setup.bash" ] && source "$HOME/so101_ws/install/setup.bash"
set -u
# shellcheck source=scripts/lib_dds.sh
source "$PROJECT_DIR/scripts/lib_dds.sh"

TOPIC=/so101_doctor_ping
PASS() { printf '  [OK]   %s\n' "$1"; }
WARN() { printf '  [!!]   %s\n' "$1"; }

# 왕복 시험은 ros2 CLI 를 쓰지 않고 rclpy 로 직접 한다.
# (ros2 topic pub/echo 는 응답하지 않는 ros2cli 데몬에 걸려 멈출 수 있어서
#  "통신이 안 된다"는 잘못된 진단을 만든다)
PUB_PY=$(cat <<'PYEOF'
import sys, rclpy
from rclpy.node import Node
from std_msgs.msg import String
rclpy.init()
n = Node('so101_doctor_pub')
p = n.create_publisher(String, sys.argv[1], 10)
n.create_timer(0.2, lambda: p.publish(String(data='ping')))
try:
    rclpy.spin(n)
except KeyboardInterrupt:
    pass
PYEOF
)
SUB_PY=$(cat <<'PYEOF'
import sys, rclpy
from rclpy.node import Node
from std_msgs.msg import String
rclpy.init()
n = Node('so101_doctor_sub')
got = []
n.create_subscription(String, sys.argv[1], lambda m: got.append(m.data), 10)
end = n.get_clock().now().nanoseconds + int(float(sys.argv[2]) * 1e9)
while not got and n.get_clock().now().nanoseconds < end:
    rclpy.spin_once(n, timeout_sec=0.2)
sys.exit(0 if got else 1)
PYEOF
)

roundtrip() {   # 서로 다른 프로세스 사이에 메시지가 오가는지 확인한다
  env "$@" python3 -c "$PUB_PY" "$TOPIC" >/dev/null 2>&1 &
  local pub=$!
  disown "$pub" 2>/dev/null || true   # kill 후 셸이 "Killed" 를 출력하지 않게
  sleep 3
  local rc=1
  env "$@" python3 -c "$SUB_PY" "$TOPIC" 8 >/dev/null 2>&1 && rc=0
  kill -9 "$pub" 2>/dev/null
  pkill -9 -f 'so101_doctor_pub' 2>/dev/null
  sleep 1
  return $rc
}

echo "===================== 1. 환경 ====================="
printf '  ROS_DISTRO=%s  RMW_IMPLEMENTATION=%s\n' "${ROS_DISTRO:-?}" "${RMW_IMPLEMENTATION:-기본(rmw_fastrtps_cpp)}"
printf '  ROS_DOMAIN_ID=%s  ROS_LOCALHOST_ONLY=%s  FASTDDS_BUILTIN_TRANSPORTS=%s\n' \
  "${ROS_DOMAIN_ID:-0}" "${ROS_LOCALHOST_ONLY:-0}" "${FASTDDS_BUILTIN_TRANSPORTS:-기본(SHM+UDP)}"

echo "===================== 2. 시계 ====================="
echo "  WSL 현재시각(UTC): $(date -u '+%Y-%m-%d %H:%M:%S')  epoch=$(date -u +%s)"
echo "  Windows 시각과 몇 분 이상 차이가 나면 PowerShell 에서: wsl --shutdown  (그리고 다시 wsl -d Ubuntu)"

echo "============== 3. 실행 중인 ROS 프로세스 =============="
if pgrep -af 'ros2_control_node|move_group|robot_state_publisher|rviz2|_ros2_daemon' >/dev/null; then
  pgrep -af 'ros2_control_node|move_group|robot_state_publisher|rviz2|_ros2_daemon' | sed 's/^/  /'
else
  echo "  없음"
fi

# ---------------------------------------------------------------------------
# 4. 여기가 이 랩탑에서 실제로 문제였던 항목이다.
#    WSL 의 networkingMode=mirrored 는 loopback0 장치와 라우팅 테이블 127 을 만들어
#    127.0.0.0/8 을 Windows 쪽 경로로 보낸다. 그 경로가 막히면 같은 PC 안의
#    프로세스끼리도 통신이 안 된다(TCP·UDP 모두). ROS 2 는 그 위에서 돌아가므로
#    "노드가 서로를 못 본다"로 나타난다.
# ---------------------------------------------------------------------------
echo "========= 4. 로컬호스트(127.0.0.1) 통신 ========="
LO_OK=1
if so101_loopback_ok; then
  PASS "127.0.0.1 UDP 왕복 정상"
else
  LO_OK=0
  WARN "127.0.0.1 로 UDP 가 오가지 않는다 (로컬호스트 자체가 막혀 있다)"
  if ip route get 127.0.0.1 2>/dev/null | grep -q loopback0; then
    WARN "원인: WSL mirrored 네트워킹. 127.0.0.1 이 loopback0 으로 우회되고 있다"
    echo "         $(ip route get 127.0.0.1 2>/dev/null | head -1)"
  fi
  for W in /mnt/c/Users/*/.wslconfig; do
    [ -f "$W" ] && grep -qi 'networkingMode *= *mirrored' "$W" \
      && echo "         $W: networkingMode=mirrored 확인"
  done
fi

echo "=========== 5. 고아 공유메모리(Fast DDS) ==========="
SHM_N=$(ls /dev/shm 2>/dev/null | grep -c '^fastrtps' || true)
if [ "$SHM_N" -gt 0 ]; then
  WARN "/dev/shm 에 fastrtps 세그먼트 $SHM_N 개. 강제 종료된 프로세스가 남긴 것이면 통신을 막는다"
else
  PASS "고아 세그먼트 없음"
fi

echo "============= 6. ros2cli 데몬 =============="
if timeout 12 ros2 daemon status >/dev/null 2>&1; then
  PASS "데몬 응답 정상"
else
  WARN "데몬이 응답하지 않는다 -> 내린다 (조회는 --no-daemon 으로 하면 된다)"
  timeout 15 ros2 daemon stop >/dev/null 2>&1 || true
fi

echo "============ 7. 프로세스 간 통신 왕복 시험 ============"
if roundtrip; then
  PASS "현재 설정으로 메시지가 오간다 - 통신 계층은 정상"
  echo
  echo "그래도 bringup 이 'Waiting for data on robot_description' 을 반복하면:"
  echo "  - 그 bringup 을 Ctrl+C 로 내리고 다시 실행해 본다 (초기화 순서 문제일 수 있다)"
  echo "  - 남아 있는 controller_manager 가 없는지 위 3번 목록을 확인한다"
  exit 0
fi
WARN "기본 설정으로는 메시지가 오가지 않는다"

# ---- 로컬호스트가 막힌 환경이면 우회 프로필로 바로 재시험한다 (sudo 불필요) ----
if [ "$LO_OK" = 0 ]; then
  echo "-- 다시 시험 (127.0.0.1 을 피하는 DDS 프로필) --"
  if so101_dds_setup && roundtrip FASTRTPS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"; then
    cat <<TXT
  [OK]   127.0.0.1 을 피해서 보내면 통신이 된다. 진단 확정:
         이 WSL 의 로컬호스트 경로(mirrored 네트워킹)가 문제이고, ROS 설정은 정상이다.

         이 프로젝트의 04·05·06·07 스크립트는 이 우회를 자동으로 적용한다.
         → 그냥 ./scripts/06_bringup.sh mock 을 실행하면 된다.

         직접 만든 터미널에서 ros2 명령을 쓸 때는 이 한 줄이 필요하다:
             export FASTRTPS_DEFAULT_PROFILES_FILE=$FASTRTPS_DEFAULT_PROFILES_FILE

         근본 해결(권장, Windows 쪽 설정 한 줄):
             1) 메모장으로 C:\\Users\\<사용자>\\.wslconfig 를 열어
                networkingMode=mirrored  줄을 지우거나 앞에 # 을 붙인다 (= 기본 NAT 모드)
             2) PowerShell:  wsl --shutdown      그리고 터미널을 다시 연다
             3) ./scripts/07_doctor.sh 로 4번 항목이 [OK] 인지 확인
         (mirrored 모드를 Windows↔WSL localhost 접속 때문에 일부러 쓰고 있다면
          그대로 두고 위 우회를 계속 쓰면 된다. 기능 차이는 없다)
TXT
    exit 0
  fi
  WARN "우회 프로필로도 안 된다"
fi

if [ "$FIX" = 0 ]; then
  cat <<'TXT'

복구하려면 (실행 중인 ROS 를 모두 내린다는 뜻이다):
  ./scripts/07_doctor.sh --fix
TXT
  exit 1
fi

echo "===================== 8. 복구 ====================="
echo "  ROS 프로세스 종료"
pkill -f ros2_control_node 2>/dev/null; pkill -f robot_state_publisher 2>/dev/null
pkill -f move_group 2>/dev/null;       pkill -f rviz2 2>/dev/null
timeout 15 ros2 daemon stop >/dev/null 2>&1
sleep 2
echo "  고아 공유메모리 정리"
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
echo "  남은 세그먼트: $(ls /dev/shm 2>/dev/null | grep -c '^fastrtps' || true)"

echo "-- 다시 시험 (정리 후, 기본 전송) --"
if roundtrip; then
  PASS "정리만으로 해결됐다. 이제 bringup 을 다시 실행한다"
  exit 0
fi

echo "-- 다시 시험 (공유메모리 없이 UDP 로만) --"
if roundtrip FASTDDS_BUILTIN_TRANSPORTS=UDPv4; then
  cat <<'TXT'
  [OK]   UDP 전용으로는 통신이 된다. 앞으로 쓰는 모든 터미널에 아래를 넣는다:

           export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

         영구 적용:  echo 'export FASTDDS_BUILTIN_TRANSPORTS=UDPv4' >> ~/.bashrc
         (06_bringup.sh 는 이미 이 설정을 기본으로 켜 둔다)
TXT
  exit 0
fi

if ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null 2>&1; then
  echo "-- 다시 시험 (CycloneDDS) --"
  if roundtrip RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; then
    cat <<'TXT'
  [OK]   CycloneDDS 로는 통신이 된다. 모든 터미널에 아래를 넣는다:

           export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

         주의: bringup 터미널과 조회 터미널이 같은 값이어야 한다.
TXT
    exit 0
  fi
fi

echo
cat <<'TXT'
  [!!]  어떤 전송 방식으로도 프로세스 간 통신이 되지 않는다.

        WSL 을 재시작한다. Windows PowerShell 에서:

            wsl --shutdown
            wsl -d Ubuntu

        그래도 안 되면 C:\Users\<사용자>\.wslconfig 에서 networkingMode=mirrored 를
        지우고 (= 기본 NAT) 다시 wsl --shutdown 한다. 자세한 근거는 docs/04 부록.
        (WSL 은 절전 후 시계가 어긋나며 DDS 타임아웃이 깨지는 일도 있다 - 재시작이 그것도 해결한다)
TXT
exit 1
