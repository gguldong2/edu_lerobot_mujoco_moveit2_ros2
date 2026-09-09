#!/usr/bin/env bash
# Build ~/so101_ws from the upstream model + this project.
#
#   ./scripts/01_setup_workspace.sh              # clone, link, patch, build (no sudo)
#   ./scripts/01_setup_workspace.sh --with-real  # + install libserial-dev and build the
#                                                #   real Feetech driver (asks for sudo)
#   ./scripts/01_setup_workspace.sh --rosdep     # + rosdep install (asks for sudo)
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${SO101_WS:-$HOME/so101_ws}"
UPSTREAM_URL="https://github.com/legalaspro/so101-ros-physical-ai.git"
RUN_ROSDEP=0
WITH_REAL=0
for a in "$@"; do
  case "$a" in
    --rosdep)    RUN_ROSDEP=1 ;;
    --with-real) WITH_REAL=1 ;;
    -h|--help)   sed -n '2,7p' "$0"; exit 0 ;;
    *) echo "unknown option: $a (see --help)" >&2; exit 2 ;;
  esac
done

[ -f /opt/ros/jazzy/setup.bash ] || { echo "ROS 2 Jazzy not found - run 00_install_jazzy.sh first" >&2; exit 1; }
# ROS 의 setup.bash 는 `set -u`(nounset) 환경에서 실패한다
#   /opt/ros/jazzy/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable
# 그래서 source 하는 동안만 -u 를 끈다.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

echo "== workspace $WS =="
mkdir -p "$WS/src" "$WS/vendor" "$WS/artifacts"

echo "== upstream model/driver =="
if [ ! -d "$WS/vendor/so101-ros-physical-ai/.git" ]; then
  git clone --recurse-submodules "$UPSTREAM_URL" "$WS/vendor/so101-ros-physical-ai"
else
  echo "  already cloned; leaving it at the current commit on purpose"
fi
UP="$WS/vendor/so101-ros-physical-ai"
git -C "$UP" rev-parse HEAD > "$WS/artifacts/so101_upstream_commit.txt"

# colcon 은 워크스페이스 아래 모든 디렉터리를 훑기 때문에 vendor/ 안의 업스트림 패키지
# (so101_bringup, so101_teleop, so101_inference, 원본 so101_moveit_config ...)까지
# 빌드 대상으로 잡는다. 우리는 src/ 의 심볼릭 링크만 쓰므로 vendor 트리는 무시하게 만든다.
touch "$WS/vendor/COLCON_IGNORE"

echo "== linking packages into src =="
ln -sfn "$UP/so101_description" "$WS/src/so101_description"
ln -sfn "$PROJECT_DIR/src/so101_project" "$WS/src/so101_project"
if [ -f "$UP/feetech_ros2_driver/package.xml" ]; then
  ln -sfn "$UP/feetech_ros2_driver" "$WS/src/feetech_ros2_driver"
  echo "  feetech_ros2_driver linked (real backend)"
else
  echo "  NOTE: feetech_ros2_driver/package.xml missing - submodule not checked out."
  echo "        git -C $UP submodule update --init --recursive"
fi

echo "== MoveIt config: copy upstream, rename, repoint controllers =="
MC="$WS/src/so101_project_moveit_config"
if [ ! -d "$MC" ]; then
  cp -r "$UP/so101_moveit_config" "$MC"
  # 1) package name (dir name alone is not enough - package.xml/CMakeLists decide)
  grep -rl 'so101_moveit_config' "$MC" | xargs -r sed -i 's/so101_moveit_config/so101_project_moveit_config/g'
  # 2) arm group name: upstream calls it "manipulator", this project uses "arm"
  grep -rl 'manipulator' "$MC" | xargs -r sed -i 's/manipulator/arm/g'
  # 3) controllers: drop the follower/ namespace, use a 1-joint JTC for the gripper
  cp "$PROJECT_DIR/src/so101_project/config/moveit_controllers.yaml" "$MC/config/moveit_controllers.yaml"
  echo "  patched $MC"
else
  echo "  $MC already exists, not touching it"
fi
echo "  leftover 'manipulator' references: $(grep -rc 'manipulator' "$MC" 2>/dev/null | grep -v ':0$' | wc -l) file(s) (want 0)"

if [ "$RUN_ROSDEP" = 1 ]; then
  echo "== rosdep (may ask for your password) =="
  rosdep install --from-paths "$WS/src" --ignore-src --rosdistro jazzy -y \
    --skip-keys "moveit_py tf_transformations mujoco_ros2_control" || \
    echo "  rosdep reported problems - the binary install from step 00 usually covers everything"
fi

echo "== colcon build =="
cd "$WS"

# 이 프로젝트가 실제로 쓰는 패키지만 빌드한다.
PKGS=(so101_description so101_project so101_project_moveit_config)

# 실물 드라이버는 libserial 이 있어야 빌드된다 (pkg_check_modules(libserial)).
if [ -e "$WS/src/feetech_ros2_driver" ]; then
  if [ "$WITH_REAL" = 1 ] && ! pkg-config --exists libserial; then
    echo "  --with-real: libserial-dev 설치 (sudo 비밀번호를 물어본다)"
    sudo apt install -y libserial-dev
  fi
  if pkg-config --exists libserial; then
    PKGS+=(feetech_ros2_driver)
    echo "  libserial 확인 -> feetech_ros2_driver 도 빌드한다"
  else
    cat <<'TXT'
  NOTE: libserial 이 없어 실물 드라이버(feetech_ros2_driver)는 이번 빌드에서 제외한다.
        시뮬레이션(mock/mujoco)에는 필요 없다. 실물을 쓰려면 둘 중 하나:
          ./scripts/01_setup_workspace.sh --with-real     (설치까지 한 번에)
          sudo apt install -y libserial-dev && ./scripts/01_setup_workspace.sh
TXT
  fi
fi

echo "  빌드 대상: ${PKGS[*]}"
colcon build --symlink-install --packages-select "${PKGS[@]}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
{
  echo "built: $(date -Is)"
  echo "upstream commit: $(cat "$WS/artifacts/so101_upstream_commit.txt")"
  dpkg-query -W 'ros-jazzy-mujoco*' 'ros-jazzy-moveit' 'ros-jazzy-ros2-control*' 2>/dev/null || true
} > "$WS/artifacts/versions.txt"

cat <<TXT

Done. Every new terminal needs:

  source /opt/ros/jazzy/setup.bash
  source $WS/install/setup.bash

Next: scripts/02_export_urdf.sh
TXT
