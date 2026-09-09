#!/usr/bin/env bash
# 화면을 보면서 제어하는 도구들. bringup 이 이미 떠 있는 다른 터미널을 전제로 한다.
#
#   ./scripts/05_gui.sh sliders    관절 슬라이더 GUI -> 궤적 컨트롤러로 전송 (제어 시연/사진용)
#   ./scripts/05_gui.sh leader     "가상 leader" 슬라이더 -> /leader/joint_states
#                                  (leader_follow.py --source topic 과 짝을 이룬다. 블로그 9편)
#   ./scripts/05_gui.sh rqt        컨트롤러 상태 GUI + 노드 그래프
#   ./scripts/05_gui.sh viewer     MuJoCo 모델만 단독으로 열기 (ROS 없이)
#   ./scripts/05_gui.sh foxglove   브라우저에서 보기 (app.foxglove.dev -> ws://localhost:8765)
set -eo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${SO101_WS:-$HOME/so101_ws}"
MODE="${1:-help}"

set +u
source /opt/ros/jazzy/setup.bash
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
set -u

# WSL 대비: bringup 과 같은 전송 설정을 쓴다 (06_bringup.sh 와 동일한 기본값)
# shellcheck source=scripts/lib_dds.sh
source "$PROJECT_DIR/scripts/lib_dds.sh"
so101_dds_setup || true
if [ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ] \
   && [ -z "${FASTDDS_BUILTIN_TRANSPORTS:-}" ] && [ -z "${RMW_IMPLEMENTATION:-}" ]; then
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
fi

have() { ros2 pkg prefix "$1" >/dev/null 2>&1; }

case "$MODE" in
  leader)
    # 블로그 9편의 leader 암 대신 슬라이더를 leader 로 쓴다. 하드웨어가 없어도 "추종"을
    # 보여줄 수 있다. /joint_states 는 이미 joint_state_broadcaster 가 쓰고 있으므로
    # 반드시 다른 토픽(/leader/joint_states)으로 리맵해야 한다.
    if ! have joint_state_publisher_gui; then
      echo "설치 필요: sudo apt install -y ros-jazzy-joint-state-publisher-gui" >&2; exit 1
    fi
    cat <<'TXT'
가상 leader 슬라이더를 연다. 이 창은 /leader/joint_states 로만 발행하며,
로봇을 직접 움직이지는 않는다. 실제 추종은 다음 노드가 담당한다(다른 터미널):

  ros2 run so101_project leader_follow.py --source topic

TXT
    # joint_state_publisher 는 /robot_description 토픽에서 모델을 읽으므로 추가 설정이 없다.
    exec ros2 run joint_state_publisher_gui joint_state_publisher_gui \
      --ros-args -r joint_states:=/leader/joint_states -p rate:=30
    ;;
  sliders)
    if ! have rqt_joint_trajectory_controller; then
      echo "설치 필요: sudo apt install -y ros-jazzy-rqt-joint-trajectory-controller" >&2; exit 1
    fi
    cat <<'TXT'
사용법:
  1) 창이 열리면 controller manager 를 /controller_manager 로 선택
  2) controller 를 arm_controller (또는 gripper_controller) 로 선택
  3) 왼쪽 빨간 전원 아이콘을 눌러 활성화하면 관절 슬라이더가 살아난다
  4) 슬라이더를 움직이면 JTC 로 궤적이 전송된다 -> MuJoCo 창에서 팔이 움직인다

주의: 이 GUI 는 MoveIt 계획을 거치지 않고 컨트롤러에 직접 보낸다(충돌 검사 없음).
      MoveIt 계획 검증은 RViz 의 MotionPlanning 패널이나 group_goal_client.py 로 한다.
      두 방법을 동시에 쓰지 않는다.
TXT
    exec ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
    ;;
  rqt)
    have rqt_controller_manager || echo "선택: sudo apt install -y ros-jazzy-rqt-controller-manager" >&2
    ros2 run rqt_controller_manager rqt_controller_manager &
    have rqt_graph && ros2 run rqt_graph rqt_graph &
    wait
    ;;
  viewer)
    SCENE="${2:-$PROJECT_DIR/src/so101_project/mjcf/scene.xml}"
    # ros-jazzy-mujoco-vendor 가 MuJoCo 공식 simulate 앱을 같이 설치한다.
    # 그걸 쓰면 pip 설치가 필요 없다(같은 프로그램, 같은 버전).
    SIM=/opt/ros/jazzy/opt/mujoco_vendor/bin/simulate
    if [ -x "$SIM" ]; then
      echo "== MuJoCo 공식 simulate 앱으로 $SCENE 열기 =="
      echo "   조작: 마우스 드래그 회전 / 휠 줌 / 더블클릭 선택 / Ctrl+드래그 밀기"
      echo "   화면 왼쪽 패널에서 Physics·Rendering 을 켜고 끌 수 있다 (F1 도움말)"
      echo "   주의: 이 창은 ROS 와 무관한 단독 시뮬레이터다. 제어는 06_bringup.sh mujoco 로 한다"
      exec "$SIM" "$SCENE"
    fi
    if [ ! -d "$HOME/venvs/mujoco" ]; then
      echo "== MuJoCo Python 환경 생성 (simulate 앱이 없어서 대체) =="
      python3 -m venv "$HOME/venvs/mujoco"
      "$HOME/venvs/mujoco/bin/pip" -q install --upgrade pip mujoco
    fi
    "$HOME/venvs/mujoco/bin/python" -c 'import mujoco; print("mujoco", mujoco.__version__)'
    echo "== $SCENE 열기 (마우스로 회전/줌, 키 F1 도움말) =="
    exec "$HOME/venvs/mujoco/bin/python" -m mujoco.viewer --mjcf="$SCENE"
    ;;
  foxglove)
    have foxglove_bridge || { echo "설치 필요: sudo apt install -y ros-jazzy-foxglove-bridge" >&2; exit 1; }
    echo "브라우저에서 https://app.foxglove.dev -> Open connection -> ws://localhost:8765"
    exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml
    ;;
  *)
    sed -n '2,11p' "$0"
    ;;
esac
