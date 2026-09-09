# shellcheck shell=bash
# 이 파일은 실행하지 않고 source 한다. "새 터미널에서 ros2 명령을 쓸 수 있게" 만드는 3단계다.
#
#   source <저장소 루트>/scripts/env.sh          (저장소 루트에서는  source scripts/env.sh )
#
# 저장소 루트에서 아래를 한 번 실행하면 새 터미널마다 자동으로 준비된다.
#   echo "source $(pwd)/scripts/env.sh" >> ~/.bashrc
# (그러면 ros2 run / ros2 topic 같은 명령을 그냥 쓸 수 있다)
#
# 하는 일:
#   1) ROS 2 Jazzy 환경         → ros2 명령이 PATH 에 들어온다
#   2) 우리 워크스페이스 환경     → so101_project 패키지를 찾을 수 있게 된다
#   3) WSL 통신 우회(lib_dds.sh) → bringup 터미널과 같은 전송 설정을 쓰게 된다
#
# 3)이 빠지면 "노드는 떠 있는데 ros2 topic list 가 비어 있다"가 된다.

_so101_env() {
  local dir ws had_u=0
  # 함수 안에서 BASH_SOURCE[0] 은 이 파일(env.sh)을 가리킨다
  dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
  ws="${SO101_WS:-$HOME/so101_ws}"

  # ROS 의 setup.bash 는 set -u 와 호환되지 않는다. 원래 상태는 되돌려 준다.
  case "$-" in *u*) had_u=1; set +u ;; esac

  if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
  else
    echo "[env] /opt/ros/jazzy 가 없다. 24.04 배포판인지 확인하라 (wsl -d Ubuntu)" >&2
    [ "$had_u" = 1 ] && set -u
    return 1
  fi

  if [ -f "$ws/install/setup.bash" ]; then
    source "$ws/install/setup.bash"
  else
    echo "[env] 워크스페이스가 없다: $ws  → 먼저 ./scripts/01_setup_workspace.sh" >&2
  fi

  [ "$had_u" = 1 ] && set -u

  if [ -f "$dir/scripts/lib_dds.sh" ]; then
    # shellcheck source=scripts/lib_dds.sh
    source "$dir/scripts/lib_dds.sh"
    so101_dds_setup || true
  fi

  export SO101_PROJECT_DIR="$dir"
  if [ -z "${SO101_QUIET:-}" ]; then
    echo "[env] ROS_DISTRO=$ROS_DISTRO  ws=$ws  준비 완료 (ros2 명령 사용 가능)"
  fi
  return 0
}
_so101_env
unset -f _so101_env
