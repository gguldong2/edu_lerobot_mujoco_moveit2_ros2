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
# 업스트림을 커밋으로 고정한다. 이 프로젝트는 업스트림의 SRDF 그룹 이름(manipulator)과
# 파일 배치를 전제로 sed 패치를 걸기 때문에, 최신 main 을 따라가면 어느 날 조용히 어긋난다.
# 최신을 쓰고 싶으면:  SO101_UPSTREAM_REF=main ./scripts/01_setup_workspace.sh
UPSTREAM_REF="${SO101_UPSTREAM_REF:-58318c905a2c61289fa907de85cb8473322fbe68}"
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
UP="$WS/vendor/so101-ros-physical-ai"
if [ ! -d "$UP/.git" ]; then
  git clone "$UPSTREAM_URL" "$UP"
  if [ "$UPSTREAM_REF" != "main" ]; then
    git -C "$UP" checkout --quiet "$UPSTREAM_REF" || {
      echo "ERROR: 업스트림 커밋 $UPSTREAM_REF 를 찾을 수 없다." >&2
      echo "       SO101_UPSTREAM_REF=main 으로 최신을 시도해 볼 수 있다(패치가 어긋날 수 있다)." >&2
      exit 1
    }
  fi
  git -C "$UP" submodule update --init --recursive || \
    echo "  NOTE: 서브모듈 체크아웃 실패 - 실물 드라이버만 영향을 받는다(시뮬레이션은 무관)"
else
  echo "  already cloned; leaving it at the current commit on purpose"
  HAVE="$(git -C "$UP" rev-parse HEAD)"
  if [ "$UPSTREAM_REF" != "main" ] && [ "$HAVE" != "$UPSTREAM_REF" ]; then
    echo "  NOTE: 고정 커밋과 다르다 (있는 것 ${HAVE:0:8}, 고정 ${UPSTREAM_REF:0:8})."
    echo "        맞추려면:  git -C $UP checkout $UPSTREAM_REF"
  fi
fi
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
  [ -d "$UP/so101_moveit_config" ] || {
    echo "ERROR: 업스트림에 so101_moveit_config 가 없다 ($UP)." >&2
    echo "       업스트림 구조가 바뀐 것이다. UPSTREAM_REF 고정값을 확인하라." >&2
    exit 1
  }
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
LEFT=$(grep -rl 'manipulator' "$MC" 2>/dev/null | wc -l)
echo "  leftover 'manipulator' references: $LEFT file(s) (want 0)"
if [ "$LEFT" != "0" ]; then
  echo "  WARNING: 그룹 이름 치환이 덜 됐다. 업스트림 구조가 바뀌었을 수 있다." >&2
fi
# 우리 launch 는 SRDF 파일 이름을 config/so101_arm.srdf 로 가정한다
[ -f "$MC/config/so101_arm.srdf" ] || \
  echo "  WARNING: $MC/config/so101_arm.srdf 가 없다. bringup 이 실패한다." >&2

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
