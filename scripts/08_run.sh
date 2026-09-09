#!/usr/bin/env bash
# 우리 노드를 "환경 준비까지 알아서" 실행한다. 새 터미널에서 바로 써도 된다.
#
#   ./scripts/08_run.sh leader_follow.py --source demo
#   ./scripts/08_run.sh joint_command.py --deg shoulder_pan=30 --time 2
#   ./scripts/08_run.sh group_goal_client.py --group arm --named rest --execute
#
# ros2 명령을 그대로 쓰고 싶을 때도 앞에 붙이면 된다:
#   ./scripts/08_run.sh ros2 control list_controllers
#   ./scripts/08_run.sh ros2 topic echo /joint_states --once
#
# 이 스크립트가 있는 이유: 새 터미널에는 ROS 환경이 없어서 그냥 ros2 를 치면
#   ros2: command not found
# 가 나온다. 매번 source 세 줄을 치는 대신 이걸 쓰면 된다.
# (영구적으로 해결하려면 ~/.bashrc 에  source <이 폴더>/scripts/env.sh  한 줄을 넣는다)
set -eo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ $# -eq 0 ]; then
  sed -n '2,18p' "$0"
  echo
  echo "실행할 수 있는 노드:"
  ls "$PROJECT_DIR/src/so101_project/scripts/" | sed 's/^/  /'
  exit 2
fi

SO101_QUIET=1
# shellcheck source=scripts/env.sh
source "$PROJECT_DIR/scripts/env.sh"

case "$1" in
  ros2)  exec "$@" ;;                       # ros2 명령 그대로 통과
  -h|--help) sed -n '2,18p' "$0"; exit 0 ;;
esac

NODE="$1"; shift
[ -f "$PROJECT_DIR/src/so101_project/scripts/$NODE" ] || NODE="$NODE.py"
if [ ! -f "$PROJECT_DIR/src/so101_project/scripts/$NODE" ]; then
  echo "그런 노드가 없다: $NODE" >&2
  ls "$PROJECT_DIR/src/so101_project/scripts/" | sed 's/^/  /' >&2
  exit 2
fi
exec ros2 run so101_project "$NODE" "$@"
