#!/usr/bin/env python3
"""Send one joint command to the controllers and report what actually happened.

This is blog post (10) - "Joint Command 역방향 경로(ROS -> 로봇)" - in this project's
shape. The blog published a single target angle on a topic; here the same idea goes out
as a **time trajectory**, because that is what a JointTrajectoryController executes and
what MoveIt produces. MoveIt is NOT involved: no collision checking, no planning.
Use this to prove the command path; use group_goal_client.py for planned motion.

    이 노드  ->  /arm_controller/follow_joint_trajectory   (액션: 결과를 받는다)
             또는 /arm_controller/joint_trajectory          (토픽: 던지고 끝)
                        |
             ros2_control (mock | mujoco | real)  ->  /joint_states  ->  RViz

Examples
--------
  # 도(deg)로 지정. 지정하지 않은 관절은 현재 위치를 유지한다
  ros2 run so101_project joint_command.py --deg shoulder_pan=30,elbow_flex=-20 --time 2.5

  # 라디안, 그리퍼 열기
  ros2 run so101_project joint_command.py --rad gripper=1.5 --time 1.0

  # 현재 위치에서 상대 이동
  ros2 run so101_project joint_command.py --deg shoulder_pan=+15 --relative

  # 블로그와 같은 "토픽으로 던지기" 방식 (결과 확인 없음)
  ros2 run so101_project joint_command.py --deg shoulder_pan=0 --via topic

  # 자기충돌 자세에 갇혀 MoveIt 이 START_STATE_IN_COLLISION 을 낼 때의 탈출구
  # (충돌 검사를 하지 않는 경로라서 갇힌 자세에서도 나올 수 있다)
  ros2 run so101_project joint_command.py --named rest --time 3

Exit codes: 0 성공 / 2 입력·한계 오류 / 3 목표 거부 / 4 시간초과 / 5 실행실패 / 6 상태 없음
"""

import argparse
import math
import sys
import xml.etree.ElementTree as ET

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)
GRIPPER = "gripper"
ARM_ORDER = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
ERR = {
    0: "SUCCESSFUL",
    -1: "INVALID_GOAL",
    -2: "INVALID_JOINTS",
    -3: "OLD_HEADER_TIMESTAMP",
    -4: "PATH_TOLERANCE_VIOLATED",
    -5: "GOAL_TOLERANCE_VIOLATED",
}


class JointCommand(Node):
    def __init__(self):
        super().__init__("joint_command")
        self.urdf = None
        self.srdf = None
        self.measured = {}
        self.create_subscription(String, "/robot_description", self._on_urdf, LATCHED)
        self.create_subscription(
            String, "/robot_description_semantic", self._on_srdf, LATCHED
        )
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

    def _on_urdf(self, msg):
        self.urdf = msg.data

    def _on_srdf(self, msg):
        self.srdf = msg.data

    def named_pose(self, name):
        """SRDF 저장 자세 -> {관절: rad}. MoveIt 을 거치지 않고도 같은 자세를 쓸 수 있게 한다."""
        if self.srdf is None:
            raise SystemExit(
                "/robot_description_semantic 을 받지 못했다. --named 는 launch_moveit:=true 인 "
                "bringup 이 필요하다 (또는 --deg/--rad 로 직접 지정한다)"
            )
        out, have = {}, []
        for gs in ET.fromstring(self.srdf).findall("group_state"):
            have.append(f"{gs.get('name')}({gs.get('group')})")
            if gs.get("name") == name:
                for j in gs.findall("joint"):
                    out[j.get("name")] = float(j.get("value"))
        if not out:
            raise SystemExit(f"저장 자세 '{name}' 가 없다. 있는 것: {', '.join(have)}")
        return out

    def _on_js(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.measured[name] = pos

    def wait_for(self, predicate, timeout, what):
        end = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while not predicate():
            if self.get_clock().now().nanoseconds > end:
                self.get_logger().error(f"{what} 를 받지 못했다. bringup 이 떠 있는가?")
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return True

    def limits(self):
        out = {}
        for j in ET.fromstring(self.urdf).findall("joint"):
            if j.get("type") in ("fixed", "floating", None):
                continue
            lim = j.find("limit")
            if lim is not None and lim.get("lower") is not None:
                out[j.get("name")] = (float(lim.get("lower")), float(lim.get("upper")))
        return out


def parse_pairs(text, to_rad):
    """'shoulder_pan=30,elbow_flex=-20' -> {joint: 값(rad)}"""
    out = {}
    for item in text.split(","):
        item = item.strip()
        if not item:
            continue
        if "=" not in item:
            raise SystemExit(f"'{item}' 형식이 잘못됐다. name=value 로 쓴다")
        name, value = item.split("=", 1)
        v = float(value.strip())
        out[name.strip()] = math.radians(v) if to_rad else v
    return out


def main():
    p = argparse.ArgumentParser(
        description="컨트롤러에 관절 명령을 직접 보낸다 (블로그 10편 대응, MoveIt 미사용)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--deg", help="도 단위 name=value 목록 (쉼표 구분)")
    p.add_argument("--rad", help="라디안 단위 name=value 목록")
    p.add_argument("--named", help="SRDF 저장 자세 이름 (rest, extended, zero, open, closed). "
                                   "충돌 검사 없이 그 자세로 직접 간다 - 막혔을 때의 탈출구")
    p.add_argument("--time", type=float, default=2.0, help="목표까지 걸릴 시간 [s]")
    p.add_argument("--relative", action="store_true", help="현재 측정값 기준 상대 이동")
    p.add_argument("--via", choices=("action", "topic"), default="action",
                   help="action: 결과·오차까지 확인 / topic: 블로그처럼 던지고 끝")
    p.add_argument("--timeout", type=float, default=30.0, help="결과 대기 [s]")
    p.add_argument("--clamp", action="store_true",
                   help="한계를 넘는 값을 거부하지 말고 한계로 잘라서 보낸다")
    p.add_argument("--show-raw", action="store_true", dest="show_raw",
                   help="같은 일을 하는 ros2 CLI 명령을 출력한다")
    args = p.parse_args()

    if not args.deg and not args.rad and not args.named:
        raise SystemExit("--deg, --rad, --named 중 하나는 있어야 한다")
    if args.time <= 0:
        raise SystemExit("--time 은 0보다 커야 한다")

    rclpy.init()
    node = JointCommand()
    rc = 0
    try:
        if not node.wait_for(lambda: node.urdf is not None, 15.0, "/robot_description"):
            return 6
        if not node.wait_for(lambda: bool(node.measured), 15.0, "/joint_states"):
            return 6
        if args.named:
            node.wait_for(lambda: node.srdf is not None, 10.0, "/robot_description_semantic")
        limits = node.limits()

        requested = {}
        if args.named:
            requested.update(node.named_pose(args.named))
        if args.deg:
            requested.update(parse_pairs(args.deg, to_rad=True))
        if args.rad:
            requested.update(parse_pairs(args.rad, to_rad=False))

        unknown = [j for j in requested if j not in limits]
        if unknown:
            node.get_logger().error(
                f"URDF 에 없는 관절: {unknown}. 가능한 관절: {sorted(limits)}"
            )
            return 2

        # 관절을 컨트롤러별로 나눈다 (controllers.yaml 과 같은 구성)
        arm_joints = sorted(
            (j for j in limits if j != GRIPPER),
            key=lambda n: ARM_ORDER.index(n) if n in ARM_ORDER else 99,
        )
        groups = {}
        if any(j != GRIPPER for j in requested):
            groups["arm_controller"] = arm_joints
        if GRIPPER in requested:
            groups["gripper_controller"] = [GRIPPER]

        # 목표값 계산: 지정하지 않은 관절은 현재 위치 유지
        targets = {}
        for joints in groups.values():
            for j in joints:
                cur = node.measured.get(j)
                if cur is None:
                    node.get_logger().error(f"{j} 의 현재 위치를 모른다")
                    return 6
                if j in requested:
                    value = requested[j]
                    want = cur + value if args.relative else value
                else:
                    want = cur
                lo, hi = limits[j]
                if want < lo - 1e-9 or want > hi + 1e-9:
                    if not args.clamp:
                        node.get_logger().error(
                            f"{j}={want:+.4f} rad 는 한계 [{lo:+.4f}, {hi:+.4f}] 를 벗어난다. "
                            "값을 고치거나 --clamp 를 쓴다"
                        )
                        return 2
                    want = min(max(want, lo), hi)
                    node.get_logger().warn(f"{j} 를 한계로 잘랐다: {want:+.4f}")
                if math.isnan(want) or math.isinf(want):
                    node.get_logger().error(f"{j} 목표가 숫자가 아니다")
                    return 2
                targets[j] = want

        dur = Duration(sec=int(args.time), nanosec=int((args.time % 1.0) * 1e9))
        for controller, joints in groups.items():
            traj = JointTrajectory()
            traj.joint_names = joints
            pt = JointTrajectoryPoint()
            pt.positions = [targets[j] for j in joints]
            pt.velocities = [0.0] * len(joints)     # 목표점에서 정지
            pt.time_from_start = dur
            traj.points = [pt]

            if args.show_raw:
                pos = ", ".join(f"{targets[j]:.4f}" for j in joints)
                print(
                    f"\n# 같은 일을 하는 CLI 명령:\n"
                    f"ros2 topic pub --once /{controller}/joint_trajectory "
                    f"trajectory_msgs/msg/JointTrajectory "
                    f"'{{joint_names: {joints}, points: [{{positions: [{pos}], "
                    f"time_from_start: {{sec: {int(args.time)}}}}}]}}'\n"
                )

            moved = {j: targets[j] for j in joints if j in requested}
            node.get_logger().info(
                f"{controller} <- {', '.join(f'{j}={v:+.4f}rad({math.degrees(v):+.1f}deg)' for j, v in moved.items())}"
                f"  ({args.time:.1f}s)"
            )

            if args.via == "topic":
                pub = node.create_publisher(
                    JointTrajectory, f"/{controller}/joint_trajectory", 10
                )
                # 구독자(컨트롤러)가 붙기 전에 발행하면 조용히 사라진다
                end = node.get_clock().now().nanoseconds + int(5e9)
                while pub.get_subscription_count() == 0:
                    if node.get_clock().now().nanoseconds > end:
                        node.get_logger().error(
                            f"/{controller}/joint_trajectory 를 듣는 컨트롤러가 없다"
                        )
                        return 3
                    rclpy.spin_once(node, timeout_sec=0.1)
                pub.publish(traj)
                for _ in range(10):
                    rclpy.spin_once(node, timeout_sec=0.05)
                node.get_logger().info("토픽으로 보냈다 (결과는 확인하지 않는다)")
                continue

            client = ActionClient(
                node, FollowJointTrajectory, f"/{controller}/follow_joint_trajectory"
            )
            if not client.wait_for_server(timeout_sec=10.0):
                node.get_logger().error(
                    f"/{controller}/follow_joint_trajectory 액션 서버가 없다. "
                    "컨트롤러가 active 인지 확인하라 (ros2 control list_controllers)"
                )
                return 3
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = traj
            send = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(node, send, timeout_sec=10.0)
            handle = send.result()
            if handle is None or not handle.accepted:
                node.get_logger().error("목표가 거부됐다")
                return 3
            result_future = handle.get_result_async()
            rclpy.spin_until_future_complete(
                node, result_future, timeout_sec=args.timeout
            )
            if result_future.result() is None:
                node.get_logger().error("결과가 오지 않았다 (시간초과)")
                handle.cancel_goal_async()
                return 4
            code = result_future.result().result.error_code
            name = ERR.get(int(code), f"UNKNOWN({code})")
            if code != 0:
                node.get_logger().error(f"{controller} 실행 실패: {name}")
                rc = 5
            else:
                node.get_logger().info(f"{controller} 실행 완료: {name}")

        # 실측 대조: "명령했다"가 아니라 "실제로 그 자리에 갔다"를 확인한다.
        # 액션 방식은 결과를 이미 기다렸고, 토픽 방식은 여기서 궤적 시간만큼 기다린다.
        settle = 0.5 if args.via == "action" else args.time + 0.5
        end = node.get_clock().now().nanoseconds + int(settle * 1e9)
        while node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        print("\n  관절            목표(rad)    실측(rad)     오차(rad)")
        worst = 0.0
        for j in sorted(targets):
            actual = node.measured.get(j, float("nan"))
            err = actual - targets[j]
            worst = max(worst, abs(err))
            print(f"  {j:<14} {targets[j]:+.4f}     {actual:+.4f}     {err:+.4f}")
        print(f"  최대 오차 {worst:.4f} rad ({math.degrees(worst):.2f} deg)\n")
        if args.via == "action" and worst > 0.05 and rc == 0:
            node.get_logger().warn(
                "오차가 크다. real 백엔드면 joint_mapping.yaml 의 sign/offset 을, "
                "mujoco 면 중력·PD 게인을 의심한다"
            )
    except KeyboardInterrupt:
        rc = 4
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
