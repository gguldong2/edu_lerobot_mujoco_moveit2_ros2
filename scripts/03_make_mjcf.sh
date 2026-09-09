#!/usr/bin/env bash
# MuJoCo physics model for the SO-101.
#
# Primary path: the model is ALREADY VENDORED in src/so101_project/mjcf/ - it comes from
# MuJoCo Menagerie (robotstudio_so101, Apache-2.0), whose joint names, ranges and
# position actuators already match this project's URDF. See mjcf/README.md.
#
# This script only (a) checks that model, or (b) runs the experimental URDF->MJCF
# converter if you want to generate one from our own URDF instead.
#
#   ./scripts/03_make_mjcf.sh            # check the vendored model
#   ./scripts/03_make_mjcf.sh --convert  # experimental converter (fallback)
set -eo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${SO101_WS:-$HOME/so101_ws}"
MJ="$PROJECT_DIR/src/so101_project/mjcf"

# ROS 의 setup.bash 는 `set -u`(nounset) 환경에서 실패한다
#   /opt/ros/jazzy/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable
# 그래서 source 하는 동안만 -u 를 끈다.
set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u

if [ "${1:-}" != "--convert" ]; then
  echo "== vendored MuJoCo model =="
  for f in scene.xml so101.xml LICENSE; do
    [ -f "$MJ/$f" ] && printf '  OK   %s\n' "$f" || { printf '  MISSING %s\n' "$f"; exit 1; }
  done
  printf '  assets: %s files\n' "$(ls "$MJ/assets" | wc -l)"

  echo "== joints / actuators (must match the URDF) =="
  python3 - "$MJ/so101.xml" <<'PY'
import sys, xml.etree.ElementTree as ET
r = ET.parse(sys.argv[1]).getroot()
js = {j.get('name'): j.get('range') for j in r.find('worldbody').iter('joint')}
print(f"  {'joint':15s}{'mjcf range':28s}actuator ctrlrange")
for a in r.find('actuator'):
    print(f"  {a.get('name'):15s}{js.get(a.get('name'),'-'):28s}{a.get('ctrlrange')}")
kf = r.find('keyframe')
print("  keyframes in so101.xml:", [k.get('name') for k in kf] if kf is not None else "(scene.xml 참고)")
PY

  echo
  echo "== open it without ROS =="
  echo "  python3 -m venv ~/venvs/mujoco && source ~/venvs/mujoco/bin/activate && pip install mujoco"
  echo "  python3 -m mujoco.viewer --mjcf=$MJ/scene.xml"
  echo
  echo "== use it as the ros2_control backend =="
  echo "  ros2 launch so101_project bringup.launch.py hardware_type:=mujoco"
  exit 0
fi

echo "== experimental converter (fallback path) =="
URDF="${2:-$WS/artifacts/so101_mock.urdf}"
OUT="${3:-$WS/artifacts/mjcf_generated}"
[ -f "$URDF" ] || { echo "no URDF at $URDF - run 02_export_urdf.sh first" >&2; exit 1; }
ros2 pkg executables mujoco_ros2_control || true
mkdir -p "$OUT"
# Fixed-base arm: no --add_free_joint.
ros2 run mujoco_ros2_control robot_description_to_mjcf.sh --save_only -u "$URDF" -o "$OUT" -c
find "$OUT" -maxdepth 2 -name '*.xml' | head
cat <<'TXT'

The official docs call this tool "hacky and _highly_ experimental!", so its output needs
hand work before use: angle units, fixed base, joint ranges, one <position> actuator per
joint with ctrlrange inside the URDF limits, positive masses, timestep ~0.002, low kp
first, and a collision-free initial keyframe. The vendored Menagerie model already has
all of that - only use this path if you specifically need a model built from our URDF.
TXT
