#!/usr/bin/env bash
# Install ROS 2 Jazzy + MoveIt 2 + ros2_control + mujoco_ros2_control.
#
# RUN THIS IN THE 24.04 DISTRO:   wsl -d Ubuntu
# It needs your sudo password, which is why it is a script you run rather than
# something the assistant executed for you.
#
#   ./scripts/00_install_jazzy.sh          전체 설치 (처음 한 번, 오래 걸린다)
#   ./scripts/00_install_jazzy.sh --gui    화면·GUI 도구만 설치/확인 (짧다, 여러 번 실행해도 안전)
#
# 이미 설치된 패키지는 apt 가 건너뛰므로 두 모드 모두 다시 실행해도 안전하다.
set -euo pipefail

GUI_ONLY=0
case "${1:-}" in
  --gui) GUI_ONLY=1 ;;
  "") ;;
  -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
  *) echo "알 수 없는 인자: $1 (사용법은 --help)" >&2; exit 2 ;;
esac

. /etc/os-release
if [ "${VERSION_CODENAME:-}" != "noble" ]; then
  echo "ERROR: this script is for Ubuntu 24.04 (noble); this shell is '${VERSION_CODENAME:-unknown}'." >&2
  echo "       Open the right distro first:  wsl -d Ubuntu" >&2
  exit 1
fi

if [ "$GUI_ONLY" = 0 ]; then
echo "== 1/5 base packages =="
# libserial-dev: 실물 Feetech 드라이버(feetech_ros2_driver)가 pkg_check_modules(libserial)로 찾는다.
#                없으면 그 패키지만 빌드에서 제외되므로 시뮬레이션에는 영향이 없다.
sudo apt update
sudo apt install -y \
  build-essential cmake git curl ca-certificates gnupg lsb-release \
  locales software-properties-common \
  python3-venv python3-dev python3-pip \
  mesa-utils libgl1 libglfw3 ffmpeg usbutils ripgrep \
  libserial-dev
sudo locale-gen en_US.UTF-8
sudo add-apt-repository -y universe

echo "== 2/5 ROS 2 apt source =="
if [ ! -f /etc/apt/sources.list.d/ros2.list ] && [ ! -f /etc/apt/sources.list.d/ros2-apt-source.list ]; then
  ROS_APT_SOURCE_VERSION="$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F '"tag_name"' | awk -F'"' '{print $4}')"
  [ -n "$ROS_APT_SOURCE_VERSION" ] || { echo "could not read ros-apt-source version" >&2; exit 1; }
  curl -fsSL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
  sudo apt update
else
  echo "  ROS apt source already present, skipping"
fi
fi   # GUI_ONLY == 0

echo "== 3/5 checking candidates before installing =="
# 없으면 프로젝트가 성립하지 않는 것들
CORE_PKGS=(
  ros-jazzy-desktop ros-dev-tools
  ros-jazzy-moveit ros-jazzy-moveit-setup-assistant ros-jazzy-pick-ik
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers
  ros-jazzy-xacro
  ros-jazzy-mujoco-ros2-control ros-jazzy-mujoco-ros2-control-demos
)
# 화면으로 보고 조작하고 촬영하기 위한 것들. 없어도 mock/mujoco 제어 자체는 된다.
#   joint-state-publisher-gui        05_gui.sh leader  (가상 leader 슬라이더, 블로그 9편)
#   rqt-joint-trajectory-controller  05_gui.sh sliders (컨트롤러 직접 구동)
#   rqt-controller-manager, rqt-graph  05_gui.sh rqt   (컨트롤러 상태·노드 그래프)
#   foxglove-bridge                  05_gui.sh foxglove (브라우저 시각화)
#   rmw-cyclonedds-cpp               07_doctor.sh 의 대체 전송 시험용
GUI_PKGS=(
  ros-jazzy-joint-state-publisher-gui
  ros-jazzy-rqt-joint-trajectory-controller ros-jazzy-rqt-controller-manager
  ros-jazzy-rqt-graph ros-jazzy-foxglove-bridge
  ros-jazzy-rmw-cyclonedds-cpp
)

candidate_of() { apt-cache policy "$1" 2>/dev/null | awk '/Candidate:/{print $2}'; }

if [ "$GUI_ONLY" = 0 ]; then
  missing=()
  for p in "${CORE_PKGS[@]}"; do
    cand="$(candidate_of "$p")"
    printf '  %-45s %s\n' "$p" "${cand:-<none>}"
    if [ -z "${cand:-}" ] || [ "$cand" = "(none)" ]; then missing+=("$p"); fi
  done
  if [ ${#missing[@]} -gt 0 ]; then
    echo "ERROR: no candidate for: ${missing[*]}" >&2
    echo "       Check the apt source step above before continuing." >&2
    exit 1
  fi
fi

# GUI 쪽은 하나가 없어도 전체를 중단하지 않는다(이름이 바뀌면 그것만 건너뛴다).
gui_install=()
gui_todo=()
for p in "${GUI_PKGS[@]}"; do
  cand="$(candidate_of "$p")"
  state="$(dpkg -s "$p" 2>/dev/null | awk '/^Status:/{print $4}')"
  printf '  %-45s %-32s %s\n' "$p" "${cand:-<none>}" "${state:-미설치}"
  if [ -n "${cand:-}" ] && [ "$cand" != "(none)" ]; then
    gui_install+=("$p")
    [ "${state:-}" = "installed" ] || gui_todo+=("$p")
  else
    echo "     ↑ apt 에 없다. 건너뛴다 (sudo apt update 후 다시 시도해 볼 수 있다)" >&2
  fi
done

if [ "$GUI_ONLY" = 1 ]; then
  if [ ${#gui_todo[@]} -eq 0 ]; then
    echo "== 이미 다 설치돼 있다. sudo 도 필요 없다 =="
  else
    echo "== GUI 도구 설치: ${gui_todo[*]} =="
    sudo apt install -y "${gui_todo[@]}"
  fi
  echo
  echo "끝. 이제 이런 것들을 쓸 수 있다:"
  echo "  ./scripts/05_gui.sh leader    가상 leader 슬라이더 (블로그 9편)"
  echo "  ./scripts/05_gui.sh sliders   컨트롤러 직접 구동"
  echo "  ./scripts/05_gui.sh rqt       컨트롤러 상태 + 노드 그래프"
  exit 0
fi

echo "== 4/5 installing (this is the long one) =="
sudo apt install -y "${CORE_PKGS[@]}" "${gui_install[@]}"

echo "== 5/5 rosdep =="
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
rosdep update

cat <<'TXT'

Done. Add this to ~/.bashrc (or source it in every ROS terminal):

  source /opt/ros/jazzy/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export ROS_DOMAIN_ID=42
  export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

Verify the MuJoCo bridge on its own before touching the SO-101:

  source /opt/ros/jazzy/setup.bash
  ros2 launch mujoco_ros2_control_demos 01_basic_robot.launch.py
  # other terminal: ros2 control list_controllers ; ros2 topic echo /joint_states --once

Next: scripts/01_setup_workspace.sh
TXT
