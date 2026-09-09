#!/usr/bin/env bash
# 스택이 제대로 배선됐는지 점검한다.
#
# 전제: **다른 터미널에서 bringup 이 돌고 있어야 한다.**
#   터미널 1:  ./scripts/06_bringup.sh mock
#   터미널 2:  ./scripts/04_smoke_test.sh
#
# 조회 명령은 모두 timeout 으로 감싸고, 가능한 곳은 --no-daemon 을 쓴다.
# (ros2cli 데몬이 WSL 에서 응답하지 않으면 traceback 이 뜨기 때문)
set -o pipefail
WS="${SO101_WS:-$HOME/so101_ws}"

set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u

# WSL 대비: bringup 과 **똑같은** 전송 설정을 써야 한다. 다르면 서로를 못 본다.
# shellcheck source=scripts/lib_dds.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib_dds.sh"
so101_dds_setup || true
if [ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ] \
   && [ -z "${FASTDDS_BUILTIN_TRANSPORTS:-}" ] && [ -z "${RMW_IMPLEMENTATION:-}" ]; then
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
fi

hr() { printf '\n--- %s ---\n' "$1"; }

# 이 스크립트는 ros2 CLI 조회를 쓰지 않고 rclpy 로 직접 그래프를 본다. 이유가 두 가지다.
#  1) 데몬을 쓰는 명령(ros2 control, ros2 daemon)은 데몬이 자기 RPC 주소를 127.0.0.1 로
#     고정하기 때문에 WSL mirrored 환경에서 30초씩 멈춘다.
#  2) --no-daemon 조회도 탐색 대기 시간이 짧아 있는 노드를 "없다"고 답하는 일이 있다.
# rclpy 는 대기 시간을 우리가 정할 수 있어서 결과가 흔들리지 않는다.

# ---------------------------------------------------------------- 사전 점검 + 노드 목록
hr "노드 (controller_manager 가 있어야 한다)"
if ! timeout 60 python3 - <<'PYEOF'
import sys
import rclpy
from rclpy.node import Node

rclpy.init()
n = Node('so101_precheck')
names = []
end = n.get_clock().now().nanoseconds + int(20e9)
while n.get_clock().now().nanoseconds < end:
    names = sorted(f"{ns.rstrip('/')}/{nm}" for nm, ns in n.get_node_names_and_namespaces())
    if any(x.endswith('/controller_manager') for x in names):
        break
    rclpy.spin_once(n, timeout_sec=0.2)
for x in names:
    print(f"  {x}")
sys.exit(0 if any(x.endswith('/controller_manager') for x in names) else 1)
PYEOF
then
  cat >&2 <<'TXT'

[중단] controller_manager 노드가 보이지 않는다.

원인은 보통 둘 중 하나다.
  1) bringup 이 안 떠 있다. 다른 터미널에서 먼저 실행한다:
         ./scripts/06_bringup.sh mock
     (이 스크립트와 bringup 은 반드시 서로 다른 터미널이어야 한다)
  2) bringup 은 떠 있는데 프로세스 간 ROS 통신이 막혔다.
         ./scripts/07_doctor.sh
TXT
  exit 1
fi
# `ros2 control ...` 은 ros2cli 데몬 경로를 타는데, 그 데몬은 자기 RPC 주소를 127.0.0.1 로
# 고정해 둔다(ros2cli/daemon/__init__.py). WSL mirrored 환경에서는 그 주소가 죽어 있어서
# 명령이 30초씩 멈춘다. 그래서 컨트롤러 조회는 서비스를 rclpy 로 직접 부른다.
hr "컨트롤러 (3개 active 여야 한다) + 하드웨어 인터페이스"
timeout 60 python3 - <<'PYEOF'
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers, ListHardwareInterfaces

rclpy.init()
n = Node('so101_cm_check')


def call(srv_type, name):
    cli = n.create_client(srv_type, name)
    if not cli.wait_for_service(timeout_sec=15.0):
        print(f"  [!!] 서비스가 없다: {name}")
        return None
    fut = cli.call_async(srv_type.Request())
    rclpy.spin_until_future_complete(n, fut, timeout_sec=15.0)
    if fut.result() is None:
        print(f"  [!!] 응답이 없다: {name}")
    return fut.result()


res = call(ListControllers, '/controller_manager/list_controllers')
if res is not None:
    for c in res.controller:
        print(f"  {c.name:<26} {c.state:<10} {c.type}")
    active = sum(1 for c in res.controller if c.state == 'active')
    print(f"  -> active {active}개 (3이어야 한다)")

res = call(ListHardwareInterfaces, '/controller_manager/list_hardware_interfaces')
if res is not None:
    cmd = [i.name for i in res.command_interfaces]
    st = [i.name for i in res.state_interfaces]
    print(f"  command interfaces {len(cmd)}개: {', '.join(sorted(cmd))}")
    print(f"  state   interfaces {len(st)}개: {', '.join(sorted(st))}")
    unclaimed = [i.name for i in res.command_interfaces if not i.is_claimed]
    if unclaimed:
        print(f"  [!!] 컨트롤러가 잡지 않은 command interface: {unclaimed}")
PYEOF
# ros2 action list / topic info / topic echo 는 직접 노드의 탐색 시간이 짧아서
# 이 환경(유니캐스트 탐색)에서는 있는 것도 "없다"고 답한다. rclpy 로 확인한다.
hr "액션 · /joint_states"
timeout 60 python3 - <<'PYEOF'
import rclpy
from rclpy.action import get_action_names_and_types
from rclpy.node import Node
from sensor_msgs.msg import JointState

rclpy.init()
n = Node('so101_graph_check')
sample = []
n.create_subscription(JointState, '/joint_states', lambda m: sample.append(m), 10)

# 탐색이 끝날 시간을 준다
end = n.get_clock().now().nanoseconds + int(5e9)
while n.get_clock().now().nanoseconds < end and not sample:
    rclpy.spin_once(n, timeout_sec=0.2)

acts = [a for a, _ in get_action_names_and_types(node=n)
        if 'follow_joint_trajectory' in a or 'move_action' in a]
print("  액션:")
for a in sorted(acts):
    print(f"    {a}")
if not acts:
    print("    [!!] 궤적/계획 액션이 보이지 않는다")

pubs = n.count_publishers('/joint_states')
print(f"  /joint_states 발행자 {pubs}개 (1이어야 한다)")
if sample:
    m = sample[-1]
    pos = ", ".join(f"{nm}={p:+.4f}" for nm, p in zip(m.name, m.position))
    print(f"  /joint_states 샘플: {pos}")
else:
    print("  [!!] /joint_states 를 받지 못했다")
PYEOF
hr "/clock (mujoco 에서만 나온다)"
# ros2 topic echo 로 보면 데몬/탐색 타이밍 때문에 있는데도 "없다"고 나오는 일이 있다.
# 그래서 rclpy 로 직접 구독해 확인한다.
timeout 20 python3 - <<'PYEOF'
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
rclpy.init()
n = Node('so101_clock_check')
got = []
n.create_subscription(Clock, '/clock', lambda m: got.append(m.clock.sec + m.clock.nanosec / 1e9), 10)
end = n.get_clock().now().nanoseconds + int(8e9)
while not got and n.get_clock().now().nanoseconds < end:
    rclpy.spin_once(n, timeout_sec=0.2)
print(f"/clock 발행됨, sim time = {got[0]:.3f} s  (mujoco 백엔드가 맞다)" if got
      else "/clock 없음 - mock/real 백엔드에서는 정상이다")
PYEOF

# ---------------------------------------------------------------- 동작 점검
hr "계획만: arm -> rest"
timeout 90 ros2 run so101_project group_goal_client.py --group arm --named rest
hr "계획만: gripper -> open"
timeout 90 ros2 run so101_project group_goal_client.py --group gripper --named open
hr "거부 확인: 그룹에 없는 관절을 넣으면 거부돼야 한다"
if timeout 90 ros2 run so101_project group_goal_client.py --group gripper --joints shoulder_pan=0.1; then
  echo "예상과 다르다: 거부되어야 하는 요청이 통과했다"
else
  echo "정상적으로 거부됨 (exit $?)"
fi

hr "직접 명령 경로 (MoveIt 없이, 블로그 10편 경로)"
timeout 90 ros2 run so101_project joint_command.py --deg shoulder_pan=10 --time 1.5 || \
  echo "직접 명령 실패 - 컨트롤러가 active 인지 위 목록을 확인한다"

cat <<'TXT'

여기까지 통과하면 실제로 움직여 본다 (느리게):
  ros2 run so101_project group_goal_client.py --group arm --named rest --execute --vel 0.1
  ros2 run so101_project group_goal_client.py --group gripper --named closed --execute

미션별 실행법(녹화·촬영 포함)은 docs/05_미션_가이드.md 에 있다.
TXT
