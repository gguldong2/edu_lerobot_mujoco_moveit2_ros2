#!/usr/bin/env bash
# Expand the project xacro for all three backends and print the joint table.
# Catches model problems before MoveIt or MuJoCo are involved.
set -euo pipefail
WS="${SO101_WS:-$HOME/so101_ws}"
# ROS 의 setup.bash 는 `set -u`(nounset) 환경에서 실패한다
#   /opt/ros/jazzy/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable
# 그래서 source 하는 동안만 -u 를 끈다.
set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u
OUT="$WS/artifacts"
mkdir -p "$OUT"
XACRO="$(ros2 pkg prefix so101_project)/share/so101_project/urdf/so101.urdf.xacro"

for HT in mock mujoco real; do
  EXTRA=()
  [ "$HT" = mujoco ] && EXTRA=("mujoco_model:=$WS/src/so101_project/mjcf/scene.xml")
  ros2 run xacro xacro "$XACRO" "hardware_type:=$HT" "${EXTRA[@]}" > "$OUT/so101_$HT.urdf"
  printf '%-7s -> %s (%s lines) plugin=%s\n' "$HT" "$OUT/so101_$HT.urdf" \
    "$(wc -l < "$OUT/so101_$HT.urdf")" \
    "$(grep -o '<plugin>[^<]*</plugin>' "$OUT/so101_$HT.urdf" | head -1 | sed 's/<[^>]*>//g')"
done

# 정석 검증 도구: urdfdom 의 check_urdf 로 트리 구조를 확인한다 (ROS 와 함께 설치됨)
if command -v check_urdf >/dev/null; then
  echo
  echo "== check_urdf =="
  check_urdf "$OUT/so101_mock.urdf" | head -30
fi

python3 - "$OUT/so101_mock.urdf" <<'PY'
import sys, xml.etree.ElementTree as ET
r = ET.parse(sys.argv[1]).getroot()
print(f"\nrobot name = {r.get('name')}   (must be so101_arm to match the reused SRDF)")
print(f"{'joint':15s}{'type':10s}{'lower':>10s}{'upper':>10s}  axis")
for j in r.findall('joint'):
    if j.get('type') == 'fixed':
        continue
    l, a = j.find('limit'), j.find('axis')
    print(f"{j.get('name'):15s}{j.get('type'):10s}{l.get('lower'):>10s}{l.get('upper'):>10s}  {a.get('xyz') if a is not None else '-'}")
rc = r.find('ros2_control')
print("\nros2_control:", rc.get('name'))
for j in rc.findall('joint'):
    print(f"  {j.get('name'):15s} cmd={[c.get('name') for c in j.findall('command_interface')]} "
          f"state={[c.get('name') for c in j.findall('state_interface')]}")
PY
echo
echo "Next: scripts/03_make_mjcf.sh (MuJoCo) or run mock straight away:"
echo "  ros2 launch so101_project bringup.launch.py hardware_type:=mock"
