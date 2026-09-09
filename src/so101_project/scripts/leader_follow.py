#!/usr/bin/env python3
"""Stream a leader's joint angles into the follower controllers.

This is the MuJoCo-era replacement for blog post (9): "leader 팔을 움직이면 follower 가
따라 움직이고 RViz 모델도 같이 움직인다". The difference is where the follower lives -
here it can be the mock backend, the MuJoCo physics model, or the real arm, and the RViz
model follows in all three because it is driven by /joint_states from
joint_state_broadcaster, not by this node.

    leader (사람/데모/실물)  ->  이 노드  ->  /arm_controller/joint_trajectory
                                          ->  /gripper_controller/joint_trajectory
                                                    |
                                        ros2_control (mock | mujoco | real)
                                                    |
                                     joint_state_broadcaster -> /joint_states -> RViz

Three leader sources
--------------------
  --source demo     내장 사인파. 하드웨어도, 추가 설치도 필요 없다 (녹화용 기본값)
  --source topic    다른 노드가 발행하는 JointState 를 따라간다
                    (예: joint_state_publisher_gui 를 /leader/joint_states 로 리맵)
  --source lerobot  실물 SO-101 leader 암을 USB 로 읽는다 (LeRobot 설치 필요)

Examples
--------
  # 아무 하드웨어 없이 추종 동작 녹화
  ros2 run so101_project leader_follow.py --source demo

  # 슬라이더를 leader 로 쓰기 (터미널 2에서 GUI, 터미널 3에서 이 노드)
  ros2 run joint_state_publisher_gui joint_state_publisher_gui \
      --ros-args -r joint_states:=/leader/joint_states
  ros2 run so101_project leader_follow.py --source topic

  # 실물 leader -> MuJoCo follower (집에서)
  ros2 run so101_project leader_follow.py --source lerobot --port /dev/ttyACM1 \
      --leader-id my_leader --print-only        # 먼저 값·방향만 확인
  ros2 run so101_project leader_follow.py --source lerobot --port /dev/ttyACM1 \
      --leader-id my_leader --flip shoulder_pan

Safety
------
* 모든 목표는 URDF 한계로 잘라낸다(clamp). 한계를 넘는 leader 값은 명령이 되지 못한다.
* 한 주기에 움직일 수 있는 양을 --max-vel 로 제한한다. leader 를 갑자기 휘둘러도
  follower 로 가는 명령은 그 속도를 넘지 않는다.
* --print-only 는 아무것도 발행하지 않는다. 실물 leader 의 부호/단위를 먼저 확인할 때 쓴다.
* MoveIt 을 거치지 않는 직접 제어이므로 충돌 검사가 없다 (블로그 9편도 같다).
  URDF 한계 안이어도 팔이 자기 몸에 접힐 수 있고, 그 자세에서는 MoveIt 이
  START_STATE_IN_COLLISION 을 낸다. 데모는 --limit-margin 으로 그것을 피하고,
  갇혔을 때는 `joint_command.py --named rest` 로 빠져나온다.
"""

import argparse
import math
import sys
import xml.etree.ElementTree as ET

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# /robot_description 은 latched(TRANSIENT_LOCAL) 로 발행된다. 나중에 붙어도 받을 수 있다.
LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)
GRIPPER = "gripper"


def urdf_limits(urdf_xml):
    """{joint: (lower, upper)} - 움직이는 관절만."""
    out = {}
    for j in ET.fromstring(urdf_xml).findall("joint"):
        if j.get("type") in ("fixed", "floating", None):
            continue
        lim = j.find("limit")
        if lim is not None and lim.get("lower") is not None:
            out[j.get("name")] = (float(lim.get("lower")), float(lim.get("upper")))
    return out


class LeaderFollow(Node):
    def __init__(self, args):
        super().__init__("leader_follow")
        self.args = args
        self.urdf = None
        self.measured = {}
        self.leader_raw = {}
        self.command = {}          # 마지막으로 보낸 명령 (속도 제한의 기준)
        self.sent = 0
        self.t0 = None

        self.create_subscription(String, "/robot_description", self._on_urdf, LATCHED)
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        if args.source == "topic":
            self.create_subscription(JointState, args.leader_topic, self._on_leader, 10)

        self.pub_arm = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10
        )
        self.pub_grip = self.create_publisher(
            JointTrajectory, "/gripper_controller/joint_trajectory", 10
        )

    # ---------------------------------------------------------------- callbacks
    def _on_urdf(self, msg):
        self.urdf = msg.data

    def _on_js(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.measured[name] = pos

    def _on_leader(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.leader_raw[name] = pos

    def wait_for(self, predicate, timeout, what):
        end = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while not predicate():
            if self.get_clock().now().nanoseconds > end:
                self.get_logger().error(
                    f"{what} 를 {timeout:.0f}초 안에 받지 못했다. bringup 이 떠 있는지 확인하라"
                )
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return True

    # ------------------------------------------------------------------- model
    def resolve_joints(self):
        limits = urdf_limits(self.urdf)
        if self.args.arm_joints:
            arm = [j.strip() for j in self.args.arm_joints.split(",") if j.strip()]
        else:
            # controllers.yaml 의 arm_controller 와 같은 집합: 움직이는 관절 - 그리퍼
            arm = [j for j in limits if j != GRIPPER]
            arm.sort(key=lambda n: _ARM_ORDER.index(n) if n in _ARM_ORDER else 99)
        unknown = [j for j in arm if j not in limits]
        if unknown:
            raise SystemExit(f"URDF 에 없는 관절: {unknown}")
        return arm, limits

    # ------------------------------------------------------------------ leader
    def leader_targets(self, now, arm, limits):
        """{joint: 목표 rad} - 소스별로 값을 만든다(단위·부호 보정 포함)."""
        a = self.args
        if a.source == "demo":
            if self.t0 is None:
                self.t0 = now
                self.center = {j: self.measured.get(j, 0.0) for j in arm + [GRIPPER]}
            t = now - self.t0
            w = 2.0 * math.pi / a.period
            out = {}
            # 관절마다 위상을 조금씩 미뤄서 팔 전체가 물결처럼 움직이게 한다(촬영용).
            # 진폭은 현재 자세에서 한계까지 남은 여유에 맞춰 줄인다(경고 없이 계속 돌게).
            for k, j in enumerate(arm):
                amp = a.amplitude * (1.0 if k < 3 else 0.6)
                lo, hi = limits[j]
                # 한계에서 --limit-margin 만큼 떨어져서 흔든다. 관절 한계 바로 앞까지 가면
                # 팔이 자기 몸에 접히면서 MoveIt 이 계획을 시작할 수 없는 자세
                # (START_STATE_IN_COLLISION)가 되기 때문이다. URDF 한계는 자기충돌을
                # 막아주지 않는다 - 그건 SRDF 의 self-collision 행렬이 하는 일이고,
                # 직접 제어 경로는 그 검사를 거치지 않는다.
                room = min(hi - self.center[j], self.center[j] - lo) - a.limit_margin
                amp = max(0.0, min(amp, room))
                out[j] = self.center[j] + amp * math.sin(w * t + k * 0.6)
            if GRIPPER in limits:
                lo, hi = a.gripper_closed, a.gripper_open
                mid, half = (hi + lo) / 2.0, (hi - lo) / 2.0
                out[GRIPPER] = mid + half * math.sin(w * t * 0.5)
            return out

        raw = dict(self.leader_raw)
        if a.source == "lerobot":
            raw = self.leader_device_read()
        out = {}
        for j, v in raw.items():
            if j not in limits:
                continue
            if j == GRIPPER:
                out[j] = self.convert_gripper(v)
            else:
                out[j] = self.convert_joint(j, v, limits[j])
        return out

    def convert_joint(self, name, value, limit):
        a = self.args
        if a.units == "deg":
            value = math.radians(value)
        elif a.units == "norm":     # -100..100 -> 관절 한계 전체 범위
            lo, hi = limit
            value = lo + (value + 100.0) * (hi - lo) / 200.0
        if name in self.flips:
            value = -value
        return value + self.offsets.get(name, 0.0)

    def convert_gripper(self, value):
        a = self.args
        if a.units == "norm":       # LeRobot 그리퍼는 0..100 인 경우가 많다
            value = a.gripper_closed + value * (a.gripper_open - a.gripper_closed) / 100.0
        elif a.units == "deg":
            value = math.radians(value)
        if GRIPPER in self.flips:
            value = -value
        return value + self.offsets.get(GRIPPER, 0.0)

    def leader_device_read(self):
        """실물 leader 암 한 번 읽기. LeRobot 은 버전마다 경로가 달라 지연 import 한다."""
        action = self.leader.get_action()
        out = {}
        for key, value in action.items():
            # LeRobot 은 "shoulder_pan.pos" 형태로 준다
            name = key.split(".")[0]
            out[name] = float(value)
        return out

    def connect_leader(self):
        errors = []
        for mod, cls, cfgcls in (
            ("lerobot.teleoperators.so101_leader", "SO101Leader", "SO101LeaderConfig"),
            ("lerobot.common.teleoperators.so101_leader", "SO101Leader", "SO101LeaderConfig"),
            ("lerobot.teleoperators.so100_leader", "SO100Leader", "SO100LeaderConfig"),
        ):
            try:
                m = __import__(mod, fromlist=[cls, cfgcls])
                Leader, Config = getattr(m, cls), getattr(m, cfgcls)
            except Exception as exc:            # noqa: BLE001 - 버전 차이를 모아서 보고
                errors.append(f"{mod}: {exc}")
                continue
            kwargs = {"port": self.args.port, "id": self.args.leader_id}
            if self.args.units == "deg":
                kwargs["use_degrees"] = True
            try:
                cfg = Config(**kwargs)
            except TypeError:                   # use_degrees 가 없는 버전
                kwargs.pop("use_degrees", None)
                cfg = Config(**kwargs)
                if self.args.units == "deg":
                    self.get_logger().warn(
                        "이 LeRobot 버전에는 use_degrees 가 없다. --units norm 이 맞을 수 있다"
                    )
            self.leader = Leader(cfg)
            self.leader.connect()
            self.get_logger().info(f"leader 연결: {mod}.{cls} port={self.args.port}")
            return
        raise SystemExit(
            "LeRobot leader 클래스를 찾지 못했다. 가상환경을 활성화했는지 확인하라:\n  "
            + "\n  ".join(errors)
        )

    # ----------------------------------------------------------------- publish
    def send(self, arm, targets, limits, dt):
        """속도 제한 + 한계 clamp 후 궤적 메시지로 보낸다."""
        a = self.args
        max_step = a.max_vel * dt
        clipped = []
        arm_point = []
        for j in arm:
            want = targets.get(j, self.command.get(j, self.measured.get(j, 0.0)))
            lo, hi = limits[j]
            if want < lo or want > hi:
                clipped.append(j)
            want = min(max(want, lo), hi)
            prev = self.command.get(j, self.measured.get(j, want))
            want = prev + max(-max_step, min(max_step, want - prev))
            self.command[j] = want
            arm_point.append(want)
        if clipped and self.sent % 30 == 0:
            self.get_logger().warn(f"한계로 잘린 관절: {clipped}")

        if not a.print_only:
            self.pub_arm.publish(self._traj(arm, arm_point))

        if GRIPPER in limits and GRIPPER in targets:
            lo, hi = limits[GRIPPER]
            want = min(max(targets[GRIPPER], lo), hi)
            prev = self.command.get(GRIPPER, self.measured.get(GRIPPER, want))
            want = prev + max(-max_step, min(max_step, want - prev))
            self.command[GRIPPER] = want
            if not a.print_only:
                self.pub_grip.publish(self._traj([GRIPPER], [want]))
        self.sent += 1

    def _traj(self, names, positions):
        msg = JointTrajectory()
        # stamp 를 0 으로 두면 컨트롤러가 "지금부터"로 해석한다.
        # (use_sim_time 인 MuJoCo 백엔드에서도 이 방식이면 시계 문제가 없다)
        msg.joint_names = list(names)
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start = Duration(
            sec=int(self.args.lookahead),
            nanosec=int((self.args.lookahead % 1.0) * 1e9),
        )
        msg.points = [pt]
        return msg

    def status_line(self, arm, limits):
        names = arm + ([GRIPPER] if GRIPPER in limits else [])
        worst, worst_j = 0.0, "-"
        for j in names:
            err = abs(self.command.get(j, 0.0) - self.measured.get(j, 0.0))
            if err > worst:
                worst, worst_j = err, j
        cmd = "  ".join(f"{SHORT.get(j, j[:6])}={self.command.get(j, 0.0):+.3f}" for j in names)
        return f"[{self.sent:5d}] {cmd}   최대추종오차 {worst:.3f} rad ({worst_j})"


_ARM_ORDER = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
# 상태줄이 한 줄에 들어가도록 줄인 이름 (녹화 화면에서 읽기 쉽게)
SHORT = {
    "shoulder_pan": "pan",
    "shoulder_lift": "lift",
    "elbow_flex": "elbow",
    "wrist_flex": "wristF",
    "wrist_roll": "wristR",
    "gripper": "grip",
}


def build_parser():
    p = argparse.ArgumentParser(
        description="leader 의 각도를 follower 컨트롤러로 흘려보낸다 (블로그 9편 대응)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--source", choices=("demo", "topic", "lerobot"), default="demo")
    p.add_argument("--rate", type=float, default=30.0, help="명령 발행 주기 [Hz]")
    p.add_argument("--lookahead", type=float, default=0.15,
                   help="각 명령의 도달 시간 [s]. 작으면 민첩, 크면 부드럽다")
    p.add_argument("--max-vel", type=float, default=2.0, dest="max_vel",
                   help="관절 최대 속도 [rad/s]. 명령을 이 속도로 제한한다")
    p.add_argument("--duration", type=float, default=0.0, help="실행 시간 [s], 0 = 무한")
    p.add_argument("--print-only", action="store_true", dest="print_only",
                   help="발행하지 않고 값만 출력 (실물 leader 부호 확인용)")
    p.add_argument("--arm-joints", dest="arm_joints",
                   help="쉼표 구분. 기본은 URDF 의 움직이는 관절 - gripper")
    # topic source
    p.add_argument("--leader-topic", default="/leader/joint_states", dest="leader_topic")
    # lerobot source
    p.add_argument("--port", default="/dev/ttyACM1", help="leader 암 USB 포트")
    p.add_argument("--leader-id", default="leader", dest="leader_id",
                   help="LeRobot 캘리브레이션 id")
    p.add_argument("--units", choices=("deg", "norm", "rad"), default=None,
                   help="leader 값의 단위. 기본값은 소스에 따라 정해진다: "
                        "topic=rad(ROS 관례), lerobot=deg. LeRobot 버전에 따라 norm(-100..100)일 수 있다")
    p.add_argument("--flip", default="", help="부호를 뒤집을 관절 (쉼표 구분)")
    p.add_argument("--offset", default="", help="관절별 오프셋 rad. 예: shoulder_pan=0.05")
    p.add_argument("--gripper-open", type=float, default=1.5, dest="gripper_open")
    p.add_argument("--gripper-closed", type=float, default=-0.16, dest="gripper_closed")
    # demo source
    p.add_argument("--period", type=float, default=8.0, help="데모 한 주기 [s]")
    p.add_argument("--amplitude", type=float, default=0.4, help="데모 진폭 [rad]")
    p.add_argument("--limit-margin", type=float, default=0.15, dest="limit_margin",
                   help="데모가 관절 한계에서 떨어져 있을 거리 [rad]. 자기충돌 자세로 "
                        "접히는 것을 막는다. 크게 흔들려면 먼저 joint_command.py "
                        "--named zero 로 여유 있는 자세로 옮긴다")
    return p


def main():
    args = build_parser().parse_args()
    if args.units is None:
        # JointState 는 ROS 관례상 라디안이다. LeRobot 은 보통 도(deg) 또는 -100..100 이다.
        args.units = "rad" if args.source == "topic" else "deg"
    rclpy.init()
    node = LeaderFollow(args)
    node.flips = {j.strip() for j in args.flip.split(",") if j.strip()}
    node.offsets = {}
    for item in args.offset.split(","):
        if item.strip():
            name, value = item.split("=", 1)
            node.offsets[name.strip()] = float(value)

    try:
        if not node.wait_for(lambda: node.urdf is not None, 15.0, "/robot_description"):
            return 6
        arm, limits = node.resolve_joints()
        if not node.wait_for(lambda: bool(node.measured), 15.0, "/joint_states"):
            return 6
        node.get_logger().info(f"arm 관절 {arm}, 그리퍼 {'있음' if GRIPPER in limits else '없음'}")

        if args.source == "lerobot":
            node.connect_leader()
        elif args.source == "topic":
            node.get_logger().info(
                f"{args.leader_topic} 를 기다린다 (leader 발행자를 켜라). 단위={args.units}"
            )
            if not node.wait_for(lambda: bool(node.leader_raw), 60.0, args.leader_topic):
                return 6

        if args.print_only:
            node.get_logger().warn("--print-only: 아무것도 발행하지 않는다")

        # 발행 주기는 시계로 직접 맞춘다. spin_once 는 메시지가 오면 바로 돌아오므로
        # 그것에 주기를 맡기면 실제 속도 제한이 --max-vel 보다 커진다.
        dt = 1.0 / args.rate
        started = node.get_clock().now().nanoseconds / 1e9
        last_send = 0.0
        next_report = 0.0
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=min(dt, 0.02))
            now = node.get_clock().now().nanoseconds / 1e9
            elapsed = now - started
            if args.duration > 0 and elapsed > args.duration:
                break
            if now - last_send < dt:
                continue
            step_dt = (now - last_send) if last_send else dt
            last_send = now
            targets = node.leader_targets(now, arm, limits)
            if targets:
                node.send(arm, targets, limits, step_dt)
            if elapsed >= next_report:
                print(node.status_line(arm, limits), flush=True)
                next_report = elapsed + 0.5
    except KeyboardInterrupt:
        print()
        node.get_logger().info("중지 (마지막 명령 위치에 그대로 멈춘다)")
    finally:
        leader = getattr(node, "leader", None)
        if leader is not None:
            try:
                leader.disconnect()
            except Exception:                   # noqa: BLE001
                pass
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
